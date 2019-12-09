# RSD 2019
## Local Logger
## Discovered dependencies:
    
```bash
sudo apt install mysql-server
sudo service mysql start
sudo mysql -u root -p
CREATE DATABASE system_log;
exit
sudo mysql -u root -p system_log < system_logger_db.sql
sudo mysql -u root -p
USE system_log;
CREATE USER 'logger'@'localhost' IDENTIFIED BY 'rsd2019';
GRANT INSERT ON system_log.* TO 'logger'@'localhost';
GRANT SELECT ON system_log.* TO 'logger'@'localhost';
exit

```