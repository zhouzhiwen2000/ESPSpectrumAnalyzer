#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -O2 -Imain \
    tests/model_test.c main/spectrum_model.c -lm -o "$build_dir/model-test"
"$build_dir/model-test"
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -O2 -Imain \
    tests/zero_span_test.c main/zero_span.c -lm -o "$build_dir/zero-span-test"
"$build_dir/zero-span-test"
