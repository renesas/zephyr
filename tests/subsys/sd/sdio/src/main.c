#include <zephyr/device.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/kernel.h>
#include <zephyr/sd/sdio.h>
#include <zephyr/ztest.h>

static const struct device *sdhc_dev = DEVICE_DT_GET(DT_ALIAS(sdhc0));
static struct sd_card card;
static struct sdio_func func1;

static K_SEM_DEFINE(sdio_irq_sem, 0, 1);

#define ESP_INT_ST      0x058
#define ESP_PKT_LEN     0x060
#define ESP_TOKEN_RDATA 0x044
#define ESP_INT_CLR     0x0D4
#define ESP_INT_ENA     0x0DC
#define ESP_FIFO_END    0x1F800

#define ESP_CMD53_LEN   512
#define ESP_BLOCK_SIZE  512

#define SDIO_THREAD_STACK_SIZE 1024
#define SDIO_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(sdio_thread_stack, SDIO_THREAD_STACK_SIZE);
static struct k_thread sdio_thread_data;
static k_tid_t sdio_thread_id;

static uint8_t tx[ESP_CMD53_LEN] __aligned(32);
static uint8_t rx[ESP_CMD53_LEN] __aligned(32);

static int esp_read32_cmd52(struct sdio_func *func, uint32_t addr, uint32_t *out)
{
	uint8_t b0, b1, b2, b3;
	int ret;

	ret = sdio_read_byte(func, addr + 0, &b0);
	if (ret) {
		return ret;
	}

	ret = sdio_read_byte(func, addr + 1, &b1);
	if (ret) {
		return ret;
	}

	ret = sdio_read_byte(func, addr + 2, &b2);
	if (ret) {
		return ret;
	}

	ret = sdio_read_byte(func, addr + 3, &b3);
	if (ret) {
		return ret;
	}

	*out = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);

	return 0;
}

static int esp_write32_cmd52(struct sdio_func *func, uint32_t addr, uint32_t v)
{
	int ret;

	ret = sdio_write_byte(func, addr + 0, (uint8_t)(v >> 0));
	if (ret) {
		return ret;
	}

	ret = sdio_write_byte(func, addr + 1, (uint8_t)(v >> 8));
	if (ret) {
		return ret;
	}

	ret = sdio_write_byte(func, addr + 2, (uint8_t)(v >> 16));
	if (ret) {
		return ret;
	}

	ret = sdio_write_byte(func, addr + 3, (uint8_t)(v >> 24));
	if (ret) {
		return ret;
	}

	return 0;
}

static void sdio_irq_cb(const struct device *dev, int status,
			const void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(status);
	ARG_UNUSED(user_data);

	sdhc_disable_interrupt(sdhc_dev, SDHC_INT_SDIO);
	k_sem_give(&sdio_irq_sem);
}

void sdio_irq_thread(void *arg1, void *arg2, void *arg3)
{
	uint32_t st;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_sem_take(&sdio_irq_sem, K_FOREVER);

		esp_read32_cmd52(&func1, 0x058, &st);

		TC_PRINT("INT_ST = 0x%08x\n", st);

		if (st & 0x1) {
			TC_PRINT("TOHOST_INT0 fired\n");
		}

		esp_write32_cmd52(&func1, 0x0D4, st);
		sdhc_enable_interrupt(sdhc_dev, sdio_irq_cb, SDHC_INT_SDIO, &card);
	}
}

ZTEST(sd_stack, test_0_init)
{
	int ret;

	zassert_true(device_is_ready(sdhc_dev), "SDHC device is not ready");

	ret = sd_is_card_present(sdhc_dev);
	zassert_equal(ret, 1, "SDIO card not present");

	ret = sd_init(sdhc_dev, &card);
	zassert_equal(ret, 0, "SDIO init failed");

	TC_PRINT("Card type:    %d\n", card.type);
	TC_PRINT("Num IO funcs: %d\n", card.num_io);
	TC_PRINT("CCCR flags:   0x%08x\n", card.cccr_flags);
	TC_PRINT("OCR:          0x%08x\n", card.ocr);

	zassert_equal(card.type, CARD_SDIO, "not SDIO card");
	zassert_true(card.num_io >= 1, "no function 1");
}

ZTEST(sd_stack, test_1_card_config)
{
	switch (card.card_voltage) {
	case SD_VOL_1_2_V:
		TC_PRINT("Card voltage: 1.2V\n");
		break;
	case SD_VOL_1_8_V:
		TC_PRINT("Card voltage: 1.8V\n");
		break;
	case SD_VOL_3_0_V:
		TC_PRINT("Card voltage: 3.0V\n");
		break;
	case SD_VOL_3_3_V:
		TC_PRINT("Card voltage: 3.3V\n");
		break;
	default:
		zassert_unreachable("Card voltage is not known value");
	}

	zassert_equal(card.status, CARD_INITIALIZED, "Card status is not OK");

	switch (card.card_speed) {
	case SD_TIMING_SDR12:
		TC_PRINT("Card timing: SDR12\n");
		break;
	case SD_TIMING_SDR25:
		TC_PRINT("Card timing: SDR25\n");
		break;
	case SD_TIMING_SDR50:
		TC_PRINT("Card timing: SDR50\n");
		break;
	case SD_TIMING_SDR104:
		TC_PRINT("Card timing: SDR104\n");
		break;
	case SD_TIMING_DDR50:
		TC_PRINT("Card timing: DDR50\n");
		break;
	default:
		zassert_unreachable("Card timing is not known value");
	}

	switch (card.type) {
	case CARD_SDIO:
		TC_PRINT("Card type: SDIO\n");
		break;
	case CARD_SDMMC:
		TC_PRINT("Card type: SDMMC\n");
		break;
	case CARD_COMBO:
		TC_PRINT("Card type: combo card\n");
		break;
	default:
		zassert_unreachable("Card type is not known value");
	}
}

ZTEST(sd_stack, test_2_read_cccr)
{
	int ret;
	uint8_t reg = 0xFF;

	ret = sdio_read_byte(&card.func0, SDIO_CCCR_CCCR, &reg);
	zassert_equal(ret, 0, "CCCR read failed");
	zassert_not_equal(reg, 0xFF, "CCCR read returned invalid data");

	TC_PRINT("CCCR = 0x%02x\n", reg);
}

ZTEST(sd_stack, test_3_write_bus_if)
{
	int ret;
	uint8_t reg;
	uint8_t rb;

	ret = sdio_read_byte(&card.func0, SDIO_CCCR_BUS_IF, &reg);
	zassert_equal(ret, 0, "read BUS_IF failed");

	ret = sdio_write_byte(&card.func0, SDIO_CCCR_BUS_IF, reg);
	zassert_equal(ret, 0, "write BUS_IF failed");

	ret = sdio_read_byte(&card.func0, SDIO_CCCR_BUS_IF, &rb);
	zassert_equal(ret, 0, "read BUS_IF after write failed");

	zassert_equal(rb, reg, "BUS_IF readback mismatch");
	TC_PRINT("BUS_IF = 0x%02x\n", rb);
}

ZTEST(sd_stack, test_4_func1_setup)
{
	int ret;
	uint8_t io_en;
	uint8_t io_rdy;
	uint8_t int_en;
	uint32_t int_st;
	uint32_t int_ena;

	sdio_thread_id = k_thread_create(&sdio_thread_data, sdio_thread_stack,
					 K_THREAD_STACK_SIZEOF(sdio_thread_stack), sdio_irq_thread,
					 NULL, NULL, NULL, SDIO_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(sdio_thread_id, "sdio_irq_thread");

	ret = sdio_init_func(&card, &func1, SDIO_FUNC_NUM_1);
	zassert_equal(ret, 0, "sdio_init_func failed");

	ret = sdio_enable_func(&func1);
	zassert_equal(ret, 0, "sdio_enable_func failed");

	ret = sdio_read_byte(&card.func0, SDIO_CCCR_IO_EN, &io_en);
	zassert_equal(ret, 0, "read IO_EN failed");
	TC_PRINT("IO_EN = 0x%02x\n", io_en);
	zassert_true(io_en & BIT(1), "func1 not enabled");

	for (int i = 0; i < 100; i++) {
		ret = sdio_read_byte(&card.func0, SDIO_CCCR_IO_RD, &io_rdy);
		zassert_equal(ret, 0, "read IO_RDY failed");

		if (io_rdy & BIT(1)) {
			break;
		}

		k_msleep(10);
	}

	TC_PRINT("IO_RDY = 0x%02x\n", io_rdy);
	zassert_true(io_rdy & BIT(1), "func1 not ready");

	ret = sdio_set_block_size(&func1, ESP_BLOCK_SIZE);
	zassert_equal(ret, 0, "set block size failed");

	ret = sdio_write_byte(&card.func0, SDIO_CCCR_INT_EN, BIT(0) | BIT(1));
	zassert_equal(ret, 0, "enable CCCR interrupt failed");

	ret = sdio_read_byte(&card.func0, SDIO_CCCR_INT_EN, &int_en);
	zassert_equal(ret, 0, "read INT_EN failed");
	TC_PRINT("CCCR INT_EN = 0x%02x\n", int_en);

	ret = esp_read32_cmd52(&func1, ESP_INT_ST, &int_st);
	zassert_equal(ret, 0, "read stale INT_ST failed");
	TC_PRINT("ESP_INT_ST before clear = 0x%08x\n", int_st);

	if (int_st != 0) {
		ret = esp_write32_cmd52(&func1, ESP_INT_CLR, int_st);
		zassert_equal(ret, 0, "clear stale INT_ST failed");
		k_msleep(5);
	}

	ret = esp_write32_cmd52(&func1, ESP_INT_ENA, BIT(0));
	zassert_equal(ret, 0, "write INT_ENA failed");

	ret = esp_read32_cmd52(&func1, ESP_INT_ENA, &int_ena);
	zassert_equal(ret, 0, "read INT_ENA failed");
	TC_PRINT("ESP_INT_ENA = 0x%08x\n", int_ena);

	ret = sdhc_enable_interrupt(sdhc_dev, sdio_irq_cb,
				    SDHC_INT_SDIO, &card);
	zassert_equal(ret, 0, "enable host SDIO interrupt failed");
}

ZTEST(sd_stack, test_5_cmd53_esp32_echo)
{
	int ret;
	uint32_t token;
	uint32_t pkt_len;
	uint32_t int_st;
	uint32_t fifo_addr;
	uint16_t free_buf_num;
	uint32_t free_capacity;
	int busy;

	zassert_not_null(func1.card, "func1 not initialized");

	busy = sdhc_card_busy(sdhc_dev);
	TC_PRINT("Card busy DAT0 low = %d\n", busy);
	zassert_equal(busy, 0, "DAT0 low before CMD53");

	for (int i = 0; i < ESP_CMD53_LEN; i++) {
		tx[i] = (uint8_t)i;
		rx[i] = 0;
	}

	ret = esp_read32_cmd52(&func1, ESP_TOKEN_RDATA, &token);
	zassert_equal(ret, 0, "read TOKEN_RDATA failed");

	free_buf_num = (token >> 16) & 0x0FFF;
	free_capacity = free_buf_num * ESP_BLOCK_SIZE;

	TC_PRINT("TOKEN_RDATA = 0x%08x\n", token);
	TC_PRINT("ESP32 RX free capacity = %u, need = %u\n",
		 free_capacity, ESP_CMD53_LEN);

	zassert_true(free_capacity >= ESP_CMD53_LEN, "ESP32 RX FIFO full");

	fifo_addr = ESP_FIFO_END - ESP_CMD53_LEN;

	TC_PRINT("CMD53 write addr = 0x%05x len = %u\n",
		 fifo_addr, ESP_CMD53_LEN);

	ret = sdio_write_addr(&func1, fifo_addr, tx, ESP_CMD53_LEN);
	zassert_equal(ret, 0, "CMD53 write failed");

	ret = k_sem_take(&sdio_irq_sem, K_MSEC(2000));
	zassert_equal(ret, 0, "timeout waiting for ESP32 interrupt");

	ret = esp_read32_cmd52(&func1, ESP_INT_ST, &int_st);
	zassert_equal(ret, 0, "read INT_ST failed");

	TC_PRINT("ESP_INT_ST = 0x%08x\n", int_st);
	zassert_true(int_st & BIT(0), "TOHOST_INT0 not set");

	ret = esp_write32_cmd52(&func1, ESP_INT_CLR, int_st);
	zassert_equal(ret, 0, "clear INT_ST failed");

	ret = esp_read32_cmd52(&func1, ESP_PKT_LEN, &pkt_len);
	zassert_equal(ret, 0, "read ESP_PKT_LEN failed");

	TC_PRINT("ESP_PKT_LEN = %u\n", pkt_len);
	zassert_true(pkt_len >= ESP_CMD53_LEN, "not enough ESP32 TX data");

	fifo_addr = ESP_FIFO_END - ESP_CMD53_LEN;

	TC_PRINT("CMD53 read addr = 0x%05x len = %u\n",
		 fifo_addr, ESP_CMD53_LEN);

	ret = sdio_read_addr(&func1, fifo_addr, rx, ESP_CMD53_LEN);
	zassert_equal(ret, 0, "CMD53 read failed");

	for (int i = 0; i < ESP_CMD53_LEN; i++) {
		zassert_equal(rx[i], tx[i],
			      "mismatch at [%d]: got 0x%02x expected 0x%02x",
			      i, rx[i], tx[i]);
	}

	TC_PRINT("CMD53 ESP32 echo passed\n");
}

ZTEST_SUITE(sd_stack, NULL, NULL, NULL, NULL, NULL);
