#!/bin/bash
# Inject graphify graph context before each prompt.
# Queries both the main (Firmware+Scripts) graph and the tests graph,
# then emits the results as additionalContext for Claude.

set -euo pipefail

PROMPT=$(jq -r '.prompt // ""' 2>/dev/null)
CWD=$(jq -r '.cwd // "."' 2>/dev/null)

# Only query if graphify is available and graph exists
GRAPH="$CWD/graphify-out/graph.json"
TESTS_GRAPH="$CWD/graphify-out/tests/graph.json"

if ! command -v graphify &>/dev/null || [ ! -f "$GRAPH" ] || [ -z "$PROMPT" ]; then
    exit 0
fi

CONTEXT=""

MAIN_RESULT=$(graphify query "$PROMPT" --graph "$GRAPH" --budget 500 2>/dev/null || true)
if [ -n "$MAIN_RESULT" ]; then
    CONTEXT="### Graphify (Firmware+Scripts)\n$MAIN_RESULT"
fi

if [ -f "$TESTS_GRAPH" ]; then
    TESTS_RESULT=$(graphify query "$PROMPT" --graph "$TESTS_GRAPH" --budget 300 2>/dev/null || true)
    if [ -n "$TESTS_RESULT" ]; then
        CONTEXT="$CONTEXT\n\n### Graphify (Tests)\n$TESTS_RESULT"
    fi
fi

if [ -z "$CONTEXT" ]; then
    exit 0
fi

jq -n --arg ctx "$(printf '%b' "$CONTEXT")" '{"additionalContext": $ctx}'
