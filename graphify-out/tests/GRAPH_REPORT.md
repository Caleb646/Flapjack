# Graph Report - Tests  (2026-05-08)

## Corpus Check
- 21 files · ~28,391 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 286 nodes · 280 edges · 18 communities (16 shown, 2 thin omitted)
- Extraction: 99% EXTRACTED · 1% INFERRED · 0% AMBIGUOUS · INFERRED: 2 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_UART Tests|UART Tests]]
- [[_COMMUNITY_IMU Tests|IMU Tests]]
- [[_COMMUNITY_Test Runner & Registry|Test Runner & Registry]]
- [[_COMMUNITY_SPI Tests|SPI Tests]]
- [[_COMMUNITY_Test Registration Script|Test Registration Script]]

## God Nodes (most connected - your core abstractions)
1. `setUpIMU()` - 6 edges
2. `generate_registration_code()` - 3 edges
3. `RegisterAllTests()` - 3 edges
4. `main()` - 3 edges
5. `find_test_functions()` - 2 edges
6. `main()` - 2 edges
7. `test_IMUInit()` - 2 edges
8. `test_IMUConf()` - 2 edges
9. `test_IMUUpdate()` - 2 edges
10. `test_IMUSelfCalibrate()` - 2 edges

## Surprising Connections (you probably didn't know these)
- `RegisterAllTests()` --calls--> `RegisterTest()`  [INFERRED]
  registered_tests.c → test_runner.c
- `main()` --calls--> `RegisterAllTests()`  [INFERRED]
  test_runner.c → registered_tests.c

## Communities (18 total, 2 thin omitted)

### Community 8 - "IMU Tests"
Cohesion: 0.27
Nodes (6): setUp(), setUpIMU(), test_IMUConf(), test_IMUInit(), test_IMUSelfCalibrate(), test_IMUUpdate()

### Community 9 - "Test Runner & Registry"
Cohesion: 0.24
Nodes (6): RegisterAllTests(), FindTestbyName(), main(), RegisterTest(), RunAllTests(), RunTestbyName()

### Community 12 - "Test Registration Script"
Cohesion: 0.83
Nodes (3): find_test_functions(), generate_registration_code(), main()

## Knowledge Gaps
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Are the 2 inferred relationships involving `RegisterAllTests()` (e.g. with `RegisterTest()` and `main()`) actually correct?**
  _`RegisterAllTests()` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Should `HAL Stubs` be split into smaller, more focused modules?**
  _Cohesion score 0.04 - nodes in this community are weakly interconnected._
- **Should `UMap Tests` be split into smaller, more focused modules?**
  _Cohesion score 0.05 - nodes in this community are weakly interconnected._
- **Should `Vector Tests` be split into smaller, more focused modules?**
  _Cohesion score 0.05 - nodes in this community are weakly interconnected._
- **Should `Queue Tests` be split into smaller, more focused modules?**
  _Cohesion score 0.07 - nodes in this community are weakly interconnected._
- **Should `UART Tests` be split into smaller, more focused modules?**
  _Cohesion score 0.08 - nodes in this community are weakly interconnected._
- **Should `Common Macro Tests` be split into smaller, more focused modules?**
  _Cohesion score 0.1 - nodes in this community are weakly interconnected._