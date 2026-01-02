#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Test script for Physical AI & Humanoid Robotics Textbook
.DESCRIPTION
    Tests backend, frontend, and RAG pipeline locally
#>

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "=" * 60 -ForegroundColor Cyan
Write-Host "Physical AI & Humanoid Robotics - Local Test" -ForegroundColor Cyan
Write-Host "=" * 60 -ForegroundColor Cyan

# Check Python
Write-Host "`n[1/6] Checking Python..." -ForegroundColor Yellow
try {
    $pyVersion = python --version 2>&1
    if ($LASTEXITCODE -ne 0) {
        $pyVersion = py --version 2>&1
    }
    Write-Host "  ✓ Python: $pyVersion" -ForegroundColor Green
} catch {
    Write-Host "  ✗ Python not found. Install Python 3.10+" -ForegroundColor Red
    exit 1
}

# Check Node.js
Write-Host "`n[2/6] Checking Node.js..." -ForegroundColor Yellow
try {
    $nodeVersion = node --version 2>&1
    Write-Host "  ✓ Node.js: $nodeVersion" -ForegroundColor Green
} catch {
    Write-Host "  ✗ Node.js not found. Install Node 18+" -ForegroundColor Red
    exit 1
}

# Check environment file
Write-Host "`n[3/6] Checking environment..." -ForegroundColor Yellow
$envFile = Join-Path $ScriptDir ".env"
if (Test-Path $envFile) {
    Write-Host "  ✓ .env file exists" -ForegroundColor Green
    Write-Host "  Note: Update with your API keys" -ForegroundColor Yellow
} else {
    Write-Host "  Copying .env.example to .env..." -ForegroundColor Yellow
    Copy-Item (Join-Path $ScriptDir ".env.example") -Destination $envFile
    Write-Host "  ✓ Created .env file" -ForegroundColor Green
    Write-Host "  ⚠ Please update with your API keys!" -ForegroundColor Red
}

# Test backend imports
Write-Host "`n[4/6] Testing backend imports..." -ForegroundColor Yellow
try {
    $backendDir = Join-Path $ScriptDir "backend"
    Push-Location $backendDir

    # Check if virtualenv exists
    if (Test-Path "venv\Scripts\Activate.ps1") {
        Write-Host "  Using virtual environment..." -ForegroundColor Yellow
    }

    # Try basic import
    $testPy = @"
import sys
sys.path.insert(0, '.')
from app.config import get_settings
print("Config OK")
from app.rag.chain import get_rag_chain
print("RAG OK")
from app.qdrant.client import get_qdrant_manager
print("Qdrant OK")
print("All imports successful")
"@
    $testResult = python -c $testPy 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  ✓ Backend imports OK" -ForegroundColor Green
    } else {
        Write-Host "  ✗ Backend import error: $testResult" -ForegroundColor Red
        Write-Host "  Note: Install dependencies with: pip install -r requirements.txt" -ForegroundColor Yellow
    }

    Pop-Location
} catch {
    Write-Host "  ⚠ Backend test skipped (install deps first)" -ForegroundColor Yellow
}

# Test frontend build
Write-Host "`n[5/6] Testing frontend build..." -ForegroundColor Yellow
try {
    Push-Location $ScriptDir

    # Check node_modules
    if (-not (Test-Path "node_modules")) {
        Write-Host "  Installing npm dependencies..." -ForegroundColor Yellow
        npm install 2>&1 | Out-Host
    }

    # Try to build
    $buildResult = npm run build 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  ✓ Frontend build OK" -ForegroundColor Green
    } else {
        Write-Host "  ⚠ Frontend build warning: $buildResult" -ForegroundColor Yellow
    }

    Pop-Location
} catch {
    Write-Host "  ⚠ Frontend test skipped" -ForegroundColor Yellow
}

# Summary
Write-Host "`n" + "=" * 60 -ForegroundColor Cyan
Write-Host "Test Summary" -ForegroundColor Cyan
Write-Host "=" * 60 -ForegroundColor Cyan

Write-Host "`nTo run the application:" -ForegroundColor White
Write-Host ""
Write-Host "  # Terminal 1 - Backend:" -ForegroundColor Yellow
Write-Host "  cd $backendDir"
Write-Host "  uvicorn app.main:app --reload --port 8000"
Write-Host ""
Write-Host "  # Terminal 2 - Frontend:" -ForegroundColor Yellow
Write-Host "  cd $ScriptDir"
Write-Host "  npm start"
Write-Host ""
Write-Host "  # Access:" -ForegroundColor Yellow
Write-Host "  - Frontend: http://localhost:3000"
Write-Host "  - API Docs: http://localhost:8000/docs"
Write-Host ""

Write-Host "`nRequired API Keys (.env):" -ForegroundColor White
Write-Host "  - OPENAI_API_KEY (for RAG)" -ForegroundColor Yellow
Write-Host "  - QDRANT_URL, QDRANT_API_KEY (for vector store)" -ForegroundColor Yellow
Write-Host "  - GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET (for auth)" -ForegroundColor Yellow
Write-Host ""

Write-Host "`nIngesting documentation:" -ForegroundColor White
Write-Host "  python scripts/ingest_docs.py" -ForegroundColor Yellow
Write-Host ""
