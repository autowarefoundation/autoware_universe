# traffic_reader

## Name

traffic_reader - monitoring network traffic by process

## Synopsis

traffic_reader [OPTION]

## Description

Monitoring network traffic by process.<br>
This runs as a daemon process and listens to a UNIX domain socket ("/tmp/traffic_reader" by default).

**Options:**<br>
_-h, --help_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Display help<br>
_-s, --socket PATH_<br>
&nbsp;&nbsp;&nbsp;&nbsp;UNIX domain socket path

**Exit status:**<br>
Returns 0 if OK; non-zero otherwise.

## Notes

The 'traffic_reader' requires nethogs command.<br>

## Operation confirmed platform

- Ubuntu 20.04.3 LTS (GNU/Linux 5.11.0-40-generic x86_64)
