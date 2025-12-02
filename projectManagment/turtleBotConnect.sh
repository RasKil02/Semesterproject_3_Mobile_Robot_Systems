sudo arp-scan -l

read -p "Indtast IP-adresse på TurtleBot: " IP

echo "Forbinder til TurtleBot på $IP..."
echo "Brugernavn: pi"
echo "Password: turtlebot"
echo ""

sshpass -p "turtlebot" ssh -t pi@$IP "bash -c '~/start_turtlebot.sh; exec bash'"

