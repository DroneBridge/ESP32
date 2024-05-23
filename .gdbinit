set $esp_reset = 0
define hook-stop
    if ($esp_reset == 0)
	set $esp_reset = 1
	printf "Reset ESP with halt command\n"
	stop
	set remote hardware-watchpoint-limit 2
	mon reset halt
	maintenance flush register-cache
	thb app_main
    end
end