#!/bin/bash
# Generate the small maize world (config/agbot_maize_small.yaml) as the "small"
# snapshot. Kept as a named entry point because CLAUDE.md and the handoff notes
# document this command; all of the work lives in generate_maize_world.sh, which
# takes any config and any snapshot name:
#
#   generate_maize_world.sh agbot_maize_long long     # the endurance world
set -e
exec "$(dirname "$0")/generate_maize_world.sh" agbot_maize_small small
