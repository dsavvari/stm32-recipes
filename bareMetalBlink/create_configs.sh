#!/bin/bash
cmake --fresh -B debug -DCMAKE_BUILD_TYPE=Debug
cmake --fresh -B release -DCMAKE_BUILD_TYPE=Release
