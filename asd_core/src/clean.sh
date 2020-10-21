#!/bin/bash

DISPLAY=:0

rm $PWD/data/*.csv $PWD/data/paths/*
python3 convert.py --no-paths