# Copyright (c) 2025 Pete Johanson
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
from unittest.mock import call, patch

import pytest
from conftest import RC_KERNEL_HEX

from runners.core import FileType
from runners.rfp import RfpBinaryRunner

TEST_RFP_PORT = 'test-rfp-serial'
TEST_RFP_PORT_SPEED = '115200'
TEST_RFP_TOOL = 'jlink'
TEST_RFP_INTERFACE = 'swd'
TEST_RFP_DEVICE = 'RA'
TEST_RFP_USR_LOCAL_RFP_CLI = '/usr/local/bin/rfp-cli'
TEST_RFP_RPD_FILE = 'test-zephyr.rpd'
TEST_RFP_TFM_RPD_FILE = 'bl2.rpd'
TEST_RFP_TFM_HEX = 'tfm_merged.hex'

EXPECTED_COMMANDS = [
    [
        'rfp-cli',
        '-port',
        TEST_RFP_PORT,
        '-device',
        TEST_RFP_DEVICE,
        '-run',
        '-noerase',
        '-p',
        '-file',
        RC_KERNEL_HEX,
    ],
]

EXPECTED_COMMANDS_WITH_SPEED = [
    [
        'rfp-cli',
        '-port',
        TEST_RFP_PORT,
        '-s',
        TEST_RFP_PORT_SPEED,
        '-device',
        TEST_RFP_DEVICE,
        '-run',
        '-noerase',
        '-p',
        '-file',
        RC_KERNEL_HEX,
    ],
]


EXPECTED_COMMANDS_WITH_ERASE = [
    [
        'rfp-cli',
        '-port',
        TEST_RFP_PORT,
        '-device',
        TEST_RFP_DEVICE,
        '-run',
        '-erase',
        '-p',
        '-file',
        RC_KERNEL_HEX,
    ],
]

EXPECTED_COMMANDS_WITH_VERIFY = [
    [
        'rfp-cli',
        '-port',
        TEST_RFP_PORT,
        '-device',
        TEST_RFP_DEVICE,
        '-run',
        '-noerase',
        '-v',
        '-p',
        '-file',
        RC_KERNEL_HEX,
    ],
]

EXPECTED_COMMANDS_WITH_RFP_CLI = [
    [
        TEST_RFP_USR_LOCAL_RFP_CLI,
        '-port',
        TEST_RFP_PORT,
        '-device',
        TEST_RFP_DEVICE,
        '-run',
        '-noerase',
        '-p',
        '-file',
        RC_KERNEL_HEX,
    ],
]

EXPECTED_COMMANDS_WITH_JLINK_HARDWARE = [
    [
        TEST_RFP_USR_LOCAL_RFP_CLI,
        '-tool',
        TEST_RFP_TOOL,
        '-interface',
        TEST_RFP_INTERFACE,
        '-device',
        TEST_RFP_DEVICE,
        '-run',
        '-erase',
        '-p',
        '-file',
        RC_KERNEL_HEX,
    ],
]

EXPECTED_COMMANDS_WITH_PARTITION_DATA = [
    [
        TEST_RFP_USR_LOCAL_RFP_CLI,
        '-tool',
        TEST_RFP_TOOL,
        '-interface',
        TEST_RFP_INTERFACE,
        '-device',
        TEST_RFP_DEVICE,
        '-fo',
        'boundary-file',
        TEST_RFP_RPD_FILE,
        '-p',
    ],
]


def require_patch(program):
    assert program in ['rfp', 'rfp-cli', TEST_RFP_USR_LOCAL_RFP_CLI]


os_path_isfile = os.path.isfile


def os_path_isfile_patch(filename):
    if filename in (RC_KERNEL_HEX, TEST_RFP_RPD_FILE, TEST_RFP_TFM_RPD_FILE, TEST_RFP_TFM_HEX):
        return True
    return os_path_isfile(filename)


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_init(dr, cc, req, runner_config, tmpdir):
    """
    Test commands using a runner created by constructor.

    Input:
        port=A
    device=B

    Output:
        -port A
        -device B
    """
    runner = RfpBinaryRunner(runner_config, port=TEST_RFP_PORT, device=TEST_RFP_DEVICE)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create(dr, cc, req, runner_config, tmpdir):
    """
    Test commands using a runner created from command line parameters.


    Input:
        ---port A
    --device B

    Output:
        -port A
        -device B
    """
    args = ['--port', str(TEST_RFP_PORT), "--device", str(TEST_RFP_DEVICE)]
    parser = argparse.ArgumentParser(allow_abbrev=False)
    RfpBinaryRunner.add_parser(parser)
    arg_namespace = parser.parse_args(args)
    runner = RfpBinaryRunner.create(runner_config, arg_namespace)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create_with_speed(dr, cc, req, runner_config, tmpdir):
    """
    Test commands using a runner created from command line parameters.

    Input:
        --port
        --speed
    --speed

    Output:
        -s SPEED
    """
    args = [
        '--device',
        str(TEST_RFP_DEVICE),
        '--port',
        str(TEST_RFP_PORT),
        '--speed',
        str(TEST_RFP_PORT_SPEED),
    ]
    parser = argparse.ArgumentParser(allow_abbrev=False)
    RfpBinaryRunner.add_parser(parser)
    arg_namespace = parser.parse_args(args)
    runner = RfpBinaryRunner.create(runner_config, arg_namespace)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS_WITH_SPEED]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create_with_erase(dr, cc, req, runner_config, tmpdir):
    """
    Test commands using a runner created from command line parameters.

    Input:
        --erase

    Output:
        -erase
    """
    args = ['--device', str(TEST_RFP_DEVICE), '--port', str(TEST_RFP_PORT), '--erase']
    parser = argparse.ArgumentParser(allow_abbrev=False)
    RfpBinaryRunner.add_parser(parser)
    arg_namespace = parser.parse_args(args)
    runner = RfpBinaryRunner.create(runner_config, arg_namespace)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS_WITH_ERASE]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create_with_verify(dr, cc, req, runner_config, tmpdir):
    """
    Test commands using a runner created from command line parameters.

    Input:
        --verify

    Output:
        -v
    """
    args = ['--device', str(TEST_RFP_DEVICE), '--port', str(TEST_RFP_PORT), '--verify']
    parser = argparse.ArgumentParser(allow_abbrev=False)
    RfpBinaryRunner.add_parser(parser)
    arg_namespace = parser.parse_args(args)
    runner = RfpBinaryRunner.create(runner_config, arg_namespace)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS_WITH_VERIFY]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create_with_rfp_cli(dr, cc, req, runner_config, tmpdir):
    """
    Test commands using a runner created from command line parameters.

    Input:
        --rfp-cli /usr/local/bin/rfp-cli

    Output:
        /usr/local/bin/rfp-cli
    """
    args = [
        '--device',
        str(TEST_RFP_DEVICE),
        '--port',
        str(TEST_RFP_PORT),
        '--rfp-cli',
        str(TEST_RFP_USR_LOCAL_RFP_CLI),
    ]
    parser = argparse.ArgumentParser(allow_abbrev=False)
    RfpBinaryRunner.add_parser(parser)
    arg_namespace = parser.parse_args(args)
    runner = RfpBinaryRunner.create(runner_config, arg_namespace)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS_WITH_RFP_CLI]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create_with_jlink_hardware(dr, cc, req, runner_config, tmpdir):
    """
    Test commands using a jlink hardware.

    Input:
        --rfp-cli /usr/local/bin/rfp-cli

    Output:
        /usr/local/bin/rfp-cli
    """
    args = [
        '--device',
        str(TEST_RFP_DEVICE),
        '--tool',
        str(TEST_RFP_TOOL),
        '--interface',
        str(TEST_RFP_INTERFACE),
        '--erase',
        '--rfp-cli',
        str(TEST_RFP_USR_LOCAL_RFP_CLI),
    ]
    parser = argparse.ArgumentParser(allow_abbrev=False)
    RfpBinaryRunner.add_parser(parser)
    arg_namespace = parser.parse_args(args)
    runner = RfpBinaryRunner.create(runner_config, arg_namespace)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS_WITH_JLINK_HARDWARE]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create_with_partition_data(dr, cc, req, runner_config, tmpdir):
    """
    Test commands using reresas partition data.

    Input:
        --rfp-cli /usr/local/bin/rfp-cli

    Output:
        /usr/local/bin/rfp-cli
    """
    args = [
        '--device',
        str(TEST_RFP_DEVICE),
        '--tool',
        str(TEST_RFP_TOOL),
        '--interface',
        str(TEST_RFP_INTERFACE),
        '--rpd-file',
        str(TEST_RFP_RPD_FILE),
        '--erase',
        '--rfp-cli',
        str(TEST_RFP_USR_LOCAL_RFP_CLI),
    ]
    parser = argparse.ArgumentParser(allow_abbrev=False)
    RfpBinaryRunner.add_parser(parser)
    arg_namespace = parser.parse_args(args)
    print(runner_config)
    runner = RfpBinaryRunner.create(runner_config, arg_namespace)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    print(cc.call_args_list)
    assert [cc.call_args_list[0]] == [call(x) for x in EXPECTED_COMMANDS_WITH_PARTITION_DATA]
    assert [cc.call_args_list[1]] == [call(x) for x in EXPECTED_COMMANDS_WITH_JLINK_HARDWARE]


EXPECTED_COMMANDS_WITH_TFM = [
    [
        'rfp-cli',
        '-tool',
        TEST_RFP_TOOL,
        '-interface',
        TEST_RFP_INTERFACE,
        '-device',
        TEST_RFP_DEVICE,
        '-fo',
        'boundary-file',
        TEST_RFP_TFM_RPD_FILE,
        '-p',
    ],
    [
        'rfp-cli',
        '-tool',
        TEST_RFP_TOOL,
        '-interface',
        TEST_RFP_INTERFACE,
        '-device',
        TEST_RFP_DEVICE,
        '-run',
        '-noerase',
        '-p',
        '-file',
        TEST_RFP_TFM_HEX,
    ],
]

EXPECTED_COMMANDS_WITH_PARTITION_DATA_PORT = [
    [
        'rfp-cli',
        '-port',
        TEST_RFP_PORT,
        '-s',
        TEST_RFP_PORT_SPEED,
        '-device',
        TEST_RFP_DEVICE,
        '-fo',
        'boundary-file',
        TEST_RFP_RPD_FILE,
        '-p',
    ],
    [
        'rfp-cli',
        '-port',
        TEST_RFP_PORT,
        '-s',
        TEST_RFP_PORT_SPEED,
        '-device',
        TEST_RFP_DEVICE,
        '-run',
        '-noerase',
        '-p',
        '-file',
        RC_KERNEL_HEX,
    ],
]


def create_runner(runner_config, args):
    parser = argparse.ArgumentParser(allow_abbrev=False)
    RfpBinaryRunner.add_parser(parser)
    arg_namespace = parser.parse_args(args)
    return RfpBinaryRunner.create(runner_config, arg_namespace)


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create_with_tfm(dr, cc, req, runner_config, tmpdir):
    """
    Test partitioning with the TF-M partition data file, then flashing the
    TF-M merged image given with --file instead of the Zephyr image.
    """
    args = [
        '--device',
        TEST_RFP_DEVICE,
        '--tool',
        TEST_RFP_TOOL,
        '--interface',
        TEST_RFP_INTERFACE,
        '--rpd-file',
        TEST_RFP_TFM_RPD_FILE,
    ]
    runner_config = runner_config._replace(file=TEST_RFP_TFM_HEX, file_type=FileType.HEX)
    runner = create_runner(runner_config, args)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS_WITH_TFM]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_create_with_partition_data_port(dr, cc, req, runner_config, tmpdir):
    """
    Test partitioning through a serial port uses the same connection as
    flashing.
    """
    args = [
        '--device',
        TEST_RFP_DEVICE,
        '--port',
        TEST_RFP_PORT,
        '--speed',
        TEST_RFP_PORT_SPEED,
        '--rpd-file',
        TEST_RFP_RPD_FILE,
    ]
    runner = create_runner(runner_config, args)
    with patch('os.path.isfile', side_effect=os_path_isfile_patch):
        runner.run('flash')
    assert cc.call_args_list == [call(x) for x in EXPECTED_COMMANDS_WITH_PARTITION_DATA_PORT]


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_missing_partition_data(dr, cc, req, runner_config, tmpdir):
    """
    Test a missing partition data file is reported before running rfp-cli.
    """
    args = ['--device', TEST_RFP_DEVICE, '--tool', TEST_RFP_TOOL, '--rpd-file', 'missing.rpd']
    runner = create_runner(runner_config, args)
    with (
        patch('os.path.isfile', side_effect=os_path_isfile_patch),
        pytest.raises(ValueError, match='missing.rpd'),
    ):
        runner.run('flash')
    assert cc.call_args_list == []


@patch('runners.core.ZephyrBinaryRunner.require', side_effect=require_patch)
@patch('runners.core.ZephyrBinaryRunner.check_call')
@patch('runners.rfp.RfpBinaryRunner.default_rfp')
def test_rfp_unsupported_file_type(dr, cc, req, runner_config, tmpdir):
    """
    Test a binary file without load address is rejected.
    """
    args = ['--device', TEST_RFP_DEVICE, '--tool', TEST_RFP_TOOL]
    runner_config = runner_config._replace(file='tfm_merged.bin', file_type=FileType.BIN)
    runner = create_runner(runner_config, args)
    with pytest.raises(ValueError, match='unsupported file type'):
        runner.run('flash')
    assert cc.call_args_list == []
