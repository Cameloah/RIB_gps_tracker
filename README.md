# RIB_gps_tracker
A device that tracks driven kilometers and transmits these to a host once wifi is available

Features:
- track kilometers in 200m intervals
- check for WiFi periodically 
- if home network available, spawn server wit driven km
- sound alarm when driving too fast in geo-fenced area

Hardware:
- ESP32 38 pin AZ-delivery board
- 12v to 3.3v DCDC converter
- GPS module
- SSR module to switch buzzer and warning light