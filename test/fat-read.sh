#!/bin/bash

# Assumes that test/fs/fs-test.sh has been run first
# FIXME: This test shows ff being /much/ faster.
# Need to extract the image from the SD card that was showing ff slower and test with that.
./sandbox/u-boot -c "host bind 0 sandbox/test/fs/3GB.fat.img; load host 0:0 0x00100000 1MB.file"
