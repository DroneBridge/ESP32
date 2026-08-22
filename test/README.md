# Native unit tests

The ESP-NOW parser regression tests are a standalone CTest project. Configure and run them with a host C compiler:

```powershell
cmake -S test -B build-unit-tests -G Ninja
cmake --build build-unit-tests
ctest --test-dir build-unit-tests --output-on-failure
```

The tests cover interleaved fragments from multiple AIR MAC addresses, sequence gaps, 32-bit sequence wraparound, and parser-table capacity.

After configuring the firmware with ESP-IDF, the same workflow is available as
a project target:

```powershell
cmake --build build --target db_esp32_unit_tests
```

This target creates a separate native test build below `build/unit-tests`, then
builds and runs it with CTest. A host C compiler is required.
