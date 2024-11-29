#!/usr/bin/env bash

file_dir=$(dirname "$(readlink -f "${0}")")
python3 -m pip install --upgrade --force-reinstall pip \
&& pip3 install -r "${file_dir}"/requirements.txt


# pip install --upgrade --force-reinstall pip \
# && pip install -r "${file_dir}"/requirements.txt
