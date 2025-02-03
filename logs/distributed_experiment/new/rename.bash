#!/bin/bash

for f in *_1.log; do mv "$f" "${f/_1.log/_5.log}"; done

