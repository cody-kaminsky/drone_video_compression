#!/usr/bin/env bash
# build.sh — run from any shell (MSYS2, Git Bash, VS Code terminal)
# Sets MSYS2 PATH and invokes make with the codec Makefile.
#
# Usage:
#   ./build.sh           — build all targets
#   ./build.sh test      — build and run all tests
#   ./build.sh clean     — clean build artifacts

export PATH="/c/msys64/mingw64/bin:/c/msys64/usr/bin:$PATH"

# gcc (a native Windows binary) uses TEMP/TMP for its temp files.
# If they point to C:\Windows\ (no write permission), compilation fails.
# Force them to the user's writable temp folder.
export TEMP='C:\Users\kamin\AppData\Local\Temp'
export TMP='C:\Users\kamin\AppData\Local\Temp'

MAKE=$(command -v make 2>/dev/null || command -v mingw32-make 2>/dev/null)
if [ -z "$MAKE" ]; then
    echo "make not found — install it with:"
    echo "  pacman -S mingw-w64-x86_64-make"
    exit 1
fi

"$MAKE" -f Makefile.codec "$@"
