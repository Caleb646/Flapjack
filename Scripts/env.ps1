# [System.EnvironmentVariableTarget]::Machine requires an administrator Powershell terminal
[System.Environment]::SetEnvironmentVariable("CLANG_TIDY_PATH", "C:/Program Files (x86)/LLVM_Clang/bin/clang-tidy", [System.EnvironmentVariableTarget]::Machine)
[System.Environment]::SetEnvironmentVariable("CLANG_FORMAT_PATH", "C:/Program Files (x86)/LLVM_Clang/bin/clang-format", [System.EnvironmentVariableTarget]::Machine)
Write-Host "Environment variables set for clang-tidy and clang-format."
