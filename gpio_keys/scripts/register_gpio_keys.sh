#!/usr/bin/env bash
set -euo pipefail

KEY1_GPIO="${1:-46}"
KEY2_GPIO="${2:-47}"

for gpio in "${KEY1_GPIO}" "${KEY2_GPIO}"; do
  if [ ! -d "/sys/class/gpio/gpio${gpio}" ]; then
    echo "${gpio}" > /sys/class/gpio/export
  fi
  echo in > "/sys/class/gpio/gpio${gpio}/direction"
  chmod 666 "/sys/class/gpio/gpio${gpio}/value"
  chmod 666 "/sys/class/gpio/gpio${gpio}/direction"
  printf 'gpio%s direction=%s value=%s\n' \
    "${gpio}" \
    "$(cat "/sys/class/gpio/gpio${gpio}/direction")" \
    "$(cat "/sys/class/gpio/gpio${gpio}/value")"
done
