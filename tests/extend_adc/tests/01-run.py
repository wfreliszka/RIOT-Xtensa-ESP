#!/usr/bin/env python3

# Copyright (C) 2018 Acutam Automation, LLC
#
# This file is subject to the terms and conditions of the GNU Lesser
# General Public License v2.1. See the file LICENSE in the top level
# directory for more details.

import os
import sys

def testfunc(child):
    child.expect_exact("ADC extension test routine")
    child.expect_exact('Number of ADC channels: ')
    c = child.readline()
    child.expect_exact('low-level channels: ')
    l = child.readline()
    child.expect_exact('extension channels: ')
    e = child.readline()
    for _ in range(int(l)):
        child.expect('Init ADC channel \d+')
        child.expect('Sample of ADC channel \d+: \d+')
    for _ in range(int(4)):
        child.expect('Init ADC channel \d+')
        child.expect('adc_ext_init dev=0xc001 chn=\d+')
        child.expect('adc_ext_sample dev=0xc001 chn=\d+ res=\d+')
        child.expect('Sample of ADC channel \d+: \d+')
    for _ in range(int(2)):
        child.expect('Init ADC channel \d+')
        child.expect('adc_ext_init dev=0xbeef chn=\d+')
        child.expect('adc_ext_sample dev=0xbeef chn=\d+ res=\d+')
        child.expect('Sample of ADC channel \d+: \d+')
    for _ in range(int(8)):
        child.expect('Init ADC channel \d+')
        child.expect('adc_ext_init dev=0x0f00 chn=\d+')
        child.expect('adc_ext_sample dev=0x0f00 chn=\d+ res=\d+')
        child.expect('Sample of ADC channel \d+: \d+')
    child.expect_exact("[SUCCESS]")


if __name__ == "__main__":
    sys.path.append(os.path.join(os.environ['RIOTBASE'], 'dist/tools/testrunner'))
    from testrunner import run
    sys.exit(run(testfunc))
