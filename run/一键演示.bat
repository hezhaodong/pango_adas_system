@echo off
setlocal
title PG2L50H ADAS Demo - One Click
echo ==========================================
echo  PG2L50H + RK3568 ADAS demo (one click)
echo  steps: self-check - flash - start - judge - shot
echo ==========================================
echo.
set "PS1=%~dp0RUN_STANDARD.ps1"
if not exist "%PS1%" (
  echo [ERROR] RUN_STANDARD.ps1 not found: %PS1%
  pause
  exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%PS1%" -Shot %*
set "RC=%ERRORLEVEL%"
echo.
if not "%RC%"=="0" (
  echo [FAIL] exit code %RC% - see FAIL lines above.
) else (
  echo %* | findstr /i "dryrun" >nul
  if errorlevel 1 (
    echo [OK] demo running and frame-rate check passed.
  ) else (
    echo [OK] dry-run finished - environment is ready.
  )
)
echo.
echo Press any key to close this window.
pause >nul
exit /b %RC%
