# CLAUDE.md

## Firmware: STM32H745 dual-core (CM7 + CM4) flight controller firmware, written in C

## Behavioral Guidelines

### 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

### 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

### 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

### 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

### 5. Use the Project's Tooling

**Never invoke a compiler, emulator or generator by hand.**

- Build / flash / sim / gen / renode → `python Scripts/board.py` — see the **flapjack-tooling** skill.
- Finding a GNC bug in the SIL → the **flapjack-sil-debugging** skill.

Skills in `.claude/skills/` are discovered automatically, so this is not a directory listing — it is the rule that you read the relevant one *before* improvising your own toolchain invocation.

### 6. Comments Earn Their Place

**Explain why, never what. The code already says what.**

Write a comment when:
- The reason isn't in the code: a convention, a spec quirk, a hardware
  constraint, a unit, a sign convention, or a bug this shape prevents.
- The obvious approach is wrong and someone will "fix" it back.

Don't write a comment when:
- It restates the line below it.
- It narrates the change ("Added X", "Now handles Y") - that's a commit message.
- It's a section banner over self-evident code.

Never commit commented-out code. Delete it; git has it.

Length follows from this: a one-line "why" is one line, a sign convention that
took a day to debug is a paragraph. Prune redundant comments in code you're
already editing - leave the rest alone (see Rule 3).

---
