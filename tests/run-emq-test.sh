#!/bin/bash
# CI's run of the emqComposite program: the image's IOC loads the composite's
# records and local stand-ins (tests/emq/config) and runs one program instance
# through a flip and an out-of-range put. Checks the output for the expected
# end state. Usage: tests/run-emq-test.sh <image tag>
set -euo pipefail
TAG=${1:?image tag}
THIS=$(realpath "$(dirname "$0")")
if docker version &> /dev/null && [[ -z ${USE_PODMAN:-} ]]; then docker=docker; else docker=podman; fi
out=$($docker run --rm --security-opt=label=disable -v "${THIS}/emq/config:/epics/ioc/config:ro" \
    "${TAG}" bash -c '${IOC}/start.sh' 2>&1 < /dev/null) || true
echo "${out}" | grep -E 'DBF_|emqComposite|^seq|Error|error' || true
check() { if ! echo "${out}" | grep -q -- "$1"; then echo "MISSING: $1"; exit 1; fi; }
if echo "${out}" | grep -q 'ERROR'; then echo "ERROR in the IOC's output (a record that did not load?)"; exit 1; fi
check 'emqComposite'
check '"polarity neg, -1.000 A"'
check '"refused: -10.500 A outside +-10"'
# after the flip the coil reads on (the negative state with NEG=1), and the
# refused out-of-range put leaves it on: the DBF lines after the flip's message
vals=$(echo "${out}" | grep -E '^DBF_' | awk '/polarity neg, -1.000 A/{f=1; next} f' | head -3 | tr -d ' ')
echo "after the flip: ${vals}"
case "${vals}" in
  'DBF_STRING:"on"'*) ;;
  *) echo "the coil did not read on after the flip"; exit 1 ;;
esac
echo "emqComposite test passed"
