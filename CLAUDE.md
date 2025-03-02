# RF Signal Scanner Development Guide

## Build Commands
```bash
# Build the project
make

# Run tests
make test

# Run a single test
make test TEST=test_name

# Lint code
make lint
```

## Code Style Guidelines
- **Formatting**: Follow Linux kernel style (tabs for indentation, max 80 chars per line)
- **Naming**: snake_case for functions/variables, UPPER_CASE for macros/constants
- **Headers**: Include guards with `#ifndef RF_SCANNER_FILE_H`
- **Error handling**: Return error codes, check all I/O operations
- **Memory**: Free all allocated memory, avoid global variables
- **Comments**: Document functions with purpose, params, and return values
- **Libraries**: Rely on librtlsdr, fftw3, and standard C libraries

## Project Organization
- `src/`: Core source code
- `include/`: Header files 
- `tests/`: Unit tests
- `tools/`: Utility scripts