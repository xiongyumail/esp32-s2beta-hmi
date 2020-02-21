local dump = require('dump')

local wifi = { _version = "0.1.0" }

function wifi.start_ap(ssid, passwd)
    net.ap(ssid, passwd)
    net.start('AP')
    httpd.start('clock')

    while (1) do
        local handle = httpd.run()
        if (handle) then
            print(string.format("event: %s, uri: %s, data: %s", handle.event, handle.uri, handle.data))
            if (handle.uri == '/config') then
                local t = dump.uri(handle.data)
                if (t.ssid and t.password) then
                    net.sta(t.ssid, t.password)
                    if (net.start('STA')) then
                        httpd.stop()
                        sys.nvs_write('wifi', 'ssid', t.ssid)
                        sys.nvs_write('wifi', 'passwd', t.password)
                        break
                    else
                        net.start('AP')
                    end
                end
            end
        end

        if (not handle) then
            sys.yield()
        end
    end
    return true
end

function wifi.start_sta(ssid, passwd)
    if (ssid and passwd) then
        net.sta(ssid, passwd)
        if (net.start('STA')) then
            sys.nvs_write('wifi', 'ssid', ssid)
            sys.nvs_write('wifi', 'passwd', passwd)
            return true
        end
    end
    
    ssid = sys.nvs_read('wifi', 'ssid')
    passwd = sys.nvs_read('wifi', 'passwd')
    
    if (ssid and passwd) then
        net.sta(ssid, passwd)
        if (net.start('STA')) then
            return true
        end
    end

    return false
end

return wifi