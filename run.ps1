Start-Process -NoNewWindow -FilePath python -ArgumentList "agent.py drone1 9101 drone2:9102 drone3:9103"
Start-Process -NoNewWindow -FilePath python -ArgumentList "agent.py drone2 9102 drone1:9101 drone3:9103"
Start-Process -NoNewWindow -FilePath python -ArgumentList "agent.py drone3 9103 drone1:9101 drone2:9102"

Write-Host "Agents launched. Press Enter to exit."
Read-Host
