#include "LakiBeamHTTP.h"

LakiBeamHTTP::LakiBeamHTTP(string local_ip, string local_port, string web_ip, string web_port):
local_ip(local_ip), local_port(local_port), web_ip(web_ip), web_port(web_port)
{
    //USED(local_ip);
    //USED(local_port);
    //USED(web_ip);
    //USED(web_port);

    check_ipconfig();
}

bool LakiBeamHTTP::get_firemware(firemware_c& firemware)
{
    bool goon = true;
    string cmdback = get_async_http(FIREMWARE_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(FIREMWARE_MODEL))
        {
            firemware.model = document[FIREMWARE_MODEL].GetString();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(FIREMWARE_SN))
        {
            firemware.sn = document[FIREMWARE_SN].GetString();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(FIREMWARE_FPGA))
        {
            firemware.fpga = document[FIREMWARE_FPGA].GetString();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(FIREMWARE_CORE))
        {
            firemware.core = document[FIREMWARE_CORE].GetString();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(FIREMWARE_AUX))
        {
            firemware.aux = document[FIREMWARE_AUX].GetString();
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_monitor(monitor_c& monitor)
{
    bool goon = true;
    string cmdback = get_async_http(MONITOR_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(MONITOR_LOAD_AVERAGE))
        {
            monitor.load_average     = document[MONITOR_LOAD_AVERAGE].GetDouble();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(MONITOR_USEAGE))
        {
            monitor.mem_useage       = document[MONITOR_USEAGE].GetDouble();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(MONITOR_UPTIME))
        {
            monitor.uptime           = document[MONITOR_UPTIME].GetDouble();
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_network(network_c &network)
{
    bool goon = true;
    string cmdback = get_async_http(NETWORK_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(NETWORK_CARRIER))
        {
            network.carrier = document[NETWORK_CARRIER].GetBool();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(NETWORK_DUPLEX))
        {
            network.duplex = document[NETWORK_DUPLEX].GetString();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(NETWORK_ETHADDR))
        {
            network.ethaddr = document[NETWORK_ETHADDR].GetString();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(NETWORK_HOSTNAME))
        {
            network.hostname = document[NETWORK_HOSTNAME].GetString();
        }
        else
        {
            goon = false;
        }

        if(document.HasMember(NETWORK_IPV4))
        {
            const value_t& value    = document[NETWORK_IPV4];
            network.ipv4_dhcp        = value[NETWORK_IPV4_DHCP].GetBool();
            int type = value[NETWORK_IPV4_ADDR].GetType();
            string str;
            if(type==5)
            {
                str = value[NETWORK_IPV4_ADDR].GetString();
                if(str.size()>0)
                {
                    network.ipv4_addr    = value[NETWORK_IPV4_ADDR].GetString();
                }
                else
                {
                    goon = false;
                }
            }
            type = value[NETWORK_IPV4_OVERRIDE].GetType();
            if(type==5)
            {
                str = value[NETWORK_IPV4_OVERRIDE].GetString();
                if(str.size()>0)
                {
                    network.ipv4_override  = value[NETWORK_IPV4_OVERRIDE].GetString();
                }
                else
                {
                    goon = false;
                }
            }
        }

        if(document.HasMember(NETWORK_SPEED))
        {
            network.speed = document[NETWORK_SPEED].GetInt();
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_override(string override, string &result)
{
    bool goon = true;

    int sieze = override.size();

    for(int i=0;i<sieze;i++)
    {
        if(((override[i]<'1')&&(override[i]!='.'))||((override[i]>'9')&&(override[i]!='.')))
        {
            goon = false;
            result = string("put_override: not ipv4 format");
        }
    }

    int cmpres = result.find_first_of("/");
    if(cmpres<0)
    {
        goon = false;
        result = string("put_override: format is wrong");
    }

    int ipver = override.find_first_of("/");
    if(ipver<=0)
    {
        goon = false;
        result = string("put_override: wrong ip format\r\n");
    }
    else
    {
        string ipaddress_tmp = override.erase(ipver);

        if(!address_t::from_string(ipaddress_tmp).is_v4())
        {
            goon = false;
            result = string("put_override: wrong ip format\r\n");
        }
        else
        {

        }
    }

    if(goon==true)
    {
        result.clear();
        string cmdback = put_async_http(OVERRIDE_CONFIG_CMD, override);
        int res = cmdback.find_first_of(OVERRIDE_CONFIG_CMD_ENSURE);
        res = strcmp(cmdback.c_str(), OVERRIDE_CONFIG_CMD_ENSURE);

        if(res>=0)
        {
            result = string(OVERRIDE_CONFIG_CMD_ENSURE);
        }
        else
        {
            goon = false;
            result = string("put_override: operation failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::delete_override(string &result)
{
    bool goon = true;

    result.clear();
    string cmdback = delete_async_http(OVERRIDE_DELETE_CMD);
    int res = cmdback.find_first_of(OVERRIDE_DELETE_CMD_ENSURE);
    res = strcmp(cmdback.c_str(), OVERRIDE_DELETE_CMD_ENSURE);

    if(res>=0)
    {
        result = string(OVERRIDE_DELETE_CMD_ENSURE);
    }
    else
    {
        goon = false;
    }

    return goon;
}

bool LakiBeamHTTP::put_reset(string &result)
{
    bool goon = true;

    result.clear();
    string cmdback = put_async_http(RESET_CMD, RESET_MESSAGE);

    int res = cmdback.find_first_of(RESET_ENSURE);
    int number = strcmp(cmdback.c_str(), RESET_ENSURE);

    if(res>=0)
    {
        cmdback.erase(res+number);
        result = string(RESET_ENSURE);
    }
    else
    {
        goon = false;
        result = string("put_reset: operation falleed");
    }

    return goon;
}

bool LakiBeamHTTP::get_overview(overview_c &overview)
{
    bool goon = true;
    string cmdback = get_async_http(OVERVIEW_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }
    if(goon==true)
    {
        if(document.HasMember(OVERVIEW_SCANFREQ))
        {
            overview.scanfreq = document[OVERVIEW_SCANFREQ].GetInt();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(OVERVIEW_MOTOR_RPM))
        {
            overview.motor_rpm = document[OVERVIEW_MOTOR_RPM].GetInt();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(OVERVIEW_LASER_ENABLE))
        {
            overview.laser_enable = document[OVERVIEW_LASER_ENABLE].GetBool();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(OVERVIEW_RESOLUTION))
        {
            overview.resolution = document[OVERVIEW_RESOLUTION].GetDouble();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(OVERVIEW_SCAN_RANGE))
        {
            const value_t& value    = document[OVERVIEW_SCAN_RANGE];
            overview.scan_range_start = value[OVERVIEW_SCAN_RANGE_START].GetInt();
            overview.scan_range_stop  = value[OVERVIEW_SCAN_RANGE_STOP].GetInt();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(OVERVIEW_FILLTER))
        {
            const value_t& value        = document[OVERVIEW_FILLTER];
            overview.filter_level       = value[OVERVIEW_FILLTER_LEVEL].GetInt();
            overview.filter_min_angle   = value[OVERVIEW_FILLTER_MIN_ANGLE].GetInt();
            overview.filter_max_angle   = value[OVERVIEW_FILLTER_MAX_ANGLE].GetInt();
            overview.filter_neighbors   = value[OVERVIEW_FILLTER_NEIGHBORS].GetInt();
            overview.window             = value[OVERVIEW_FILLTER_WINDOW].GetInt();
        }
        else
        {
            goon = false;
        }
        if(document.HasMember(OVERVIEW_HOST))
        {
            const value_t& value    = document[OVERVIEW_HOST];
            overview.host_ip = value[OVERVIEW_HOST_IP].GetString();
            overview.host_port  = value[OVERVIEW_HOST_PORT].GetInt();
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_scanfreq(string& freq)
{
    bool goon = true;

    int freq_tep = atoi(freq.c_str());

    switch (freq_tep)
    {
    case 10:
        break;
    case 20:
        break;
    case 25:
        break;
    case 30:
        break;
    default:
        goon = false;
        freq = string("put_scanfreq: wrong freq");
        break;
    }

    if(goon==true)
    {
        freq = to_string(freq_tep);

        string cmdback = put_async_http(SCANFREQ_CONFIG_CMD, freq);
        int res = cmdback.find_first_of(SCANFREQ_CONFIG_ENSURE);
        res = strcmp(cmdback.c_str(), SCANFREQ_CONFIG_ENSURE);

        if(!(res>=0))
        {
            goon = false;
            freq = string("put_scanfreq: operate failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_scanfreq(string &result)
{
    bool goon = true;
    string cmdback = get_async_http(SCANFREQ_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }
    if(goon==true)
    {
        if(document.HasMember(SCANFREQ_FREQ))
        {
            int output_freq;
            output_freq = document[SCANFREQ_FREQ].GetInt();
            result = std::to_string(output_freq);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_motor_rpm(string &motor_rpm)
{
    bool goon = true;
    string cmdback = get_async_http(MOTOR_RPM_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(MOTOR_RPM_RPM))
        {
            int output_rpm;
            output_rpm = document[MOTOR_RPM_RPM].GetInt();
            motor_rpm = std::to_string(output_rpm);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_laser_enable(string &laser_state)
{
    bool goon = true;
    string cmdback = get_async_http(LASER_ENABLE_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(LASER_ENABLE_ENABLE))
        {
            goon = document[LASER_ENABLE_ENABLE].GetBool();
        }
        else
        {
            goon = false;
        }
    }

    if(goon==true)
    {
        laser_state = string("true");
    }
    else
    {
        laser_state = string("false");
    }

    return goon;
}

bool LakiBeamHTTP::put_laser_enable(string &config_state)
{
    bool goon = true;

    if(config_state=="true")
    {
        config_state = "true";
    }
    else if(config_state=="false")
    {
        config_state = "false";
    }
    else
    {
        goon = false;
        config_state = string("put_laser_enable: wrong parameter");
    }

    if(goon==true)
    {
        string cmdback = put_async_http(LASER_ENABLE_CMD, config_state);
        int res = cmdback.find_first_of(LASER_ENABLE_ENSURE);
        res = strcmp(cmdback.c_str(), LASER_ENABLE_ENSURE);

        if(!(res>=0))
        {
            goon = false;
            config_state = string("put_laser_enable: operation failed");
        }
        else
        {
            config_state = "true";
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_resolution(string &resolution)
{
    bool goon = true;
    string cmdback = get_async_http(LASER_SESOLUTION_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(LASER_SESOLUTION_SESOLUTION))
        {
            double output_rpm;
            output_rpm = document[LASER_SESOLUTION_SESOLUTION].GetDouble();
            resolution = std::to_string(output_rpm);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_scan_range(scan_range_c &scan_range)
{
    bool goon = true;
    string cmdback = get_async_http(SCAN_RANGE_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(SCAN_RANGE_RANGE))
        {
            const value_t& value    = document[SCAN_RANGE_RANGE];
            scan_range.start        = value[SCAN_RANGE_START].GetInt();
            scan_range.stop         = value[SCAN_RANGE_STOP].GetInt();
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_laser_start(string &start)
{
    bool goon = true;
    string cmdback = get_async_http(START_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(START_START))
        {
            int output_freq;
            output_freq = document[START_START].GetInt();
            start = std::to_string(output_freq);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_laser_stop(string &stop)
{
    bool goon = true;
    string cmdback = get_async_http(STOP_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(STOP_STOP))
        {
            int output_freq;
            output_freq = document[STOP_STOP].GetInt();
            stop = std::to_string(output_freq);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_laser_start(string &start)
{
    bool goon = true;

    u16_t startfreq = atoi(start.c_str());
    string stop;

    if(startfreq>315)
    {
        goon = false;
        start = string("put_laser_start: start must < 315 and > 45");
    }
    if(startfreq<45)
    {
        goon = false;
        start = string("put_laser_start: start must < 315 and > 45");
    }

    if(goon==true)
    {
        get_laser_stop(stop);
        u16_t stopfreq = atoi(stop.c_str());
        if(startfreq>=stopfreq)
        {
            goon = false;
            start = string("put_laser_start: start must > stop");
        }
    }

    if(goon==true)
    {
        start = to_string(startfreq);

        string cmdback = put_async_http(START_PUT_CMD, start);
        int res = cmdback.find_first_of(START_PUT_CMD_ENSURE);
        res = strcmp(cmdback.c_str(), START_PUT_CMD_ENSURE);

        if(!(res>=0))
        {
            goon = false;
            start = string("put_laser_start: operation failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_laser_stop(string &stop)
{
    bool goon = true;

    u16_t stopfreq = atoi(stop.c_str());
    string start;

    if(stopfreq>315)
    {
        goon = false;
        start = string("put_laser_stop: stop must < 315 and > 45");
    }
    if(stopfreq<45)
    {
        goon = false;
        start = string("put_laser_stop: stop must < 315 and > 45");
    }
    if(goon==true)
    {
        get_laser_start(start);
        u16_t startfreq = atoi(start.c_str());
        if(startfreq>=stopfreq)
        {
            goon = false;
            stop = string("put_laser_stop: start must > stop");
        }
    }

    if(goon==true)
    {
        string cmdback = put_async_http(STOP_PUT_CMD, stop);
        int res = cmdback.find_first_of(STOP_PUT_CMD_ENSURE);
        res = strcmp(cmdback.c_str(), STOP_PUT_CMD_ENSURE);

        if(!(res>=0))
        {
            goon = false;
            stop = string("put_laser_stop: operation failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_filter_level(filter_c &level)
{
    bool goon = true;
    string cmdback = get_async_http(FILTER_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(FILTER_FILTER))
        {
            const value_t& value    = document[FILTER_FILTER];
            level.level             = value[FILTER_LEVEL].GetInt();
            level.window            = value[FILTER_WINDOW].GetInt();
            level.min_angle         = value[FILTER_MIN_ANGLE].GetInt();
            level.max_angle         = value[FILTER_MAX_ANGLE].GetInt();
            level.neighbors         = value[FILTER_NEIGHBORS].GetInt();
            cout << level.level << endl;
            cout << level.window << endl;
            cout << level.min_angle << endl;
            cout << level.max_angle << endl;
            cout << level.neighbors << endl;
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_filter_level(string &level)
{
    bool goon = true;

    int level_tep = atoi(level.c_str());

    switch (level_tep) {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    default:
        goon = false;
        level = string("put_filter_level: wrong level parameter");
        break;

    }

    if(goon==true)
    {
        level = to_string(level_tep);

        string cmdback = put_async_http(FILTER_LEVEL_CMD, level);
        int res = cmdback.find_first_of(FILTER_LEVEL_ENSURE);
        res = strcmp(cmdback.c_str(), FILTER_LEVEL_ENSURE);

        if(!(res>=0))
        {
            goon = false;
            level = string("put_filter_level: operation failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_filter_window(string &window)
{
    bool goon = true;
    string cmdback = get_async_http(FILTER_WINDOW_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(FILTER_WINDOW_WINDOW))
        {
            int temp = document[FILTER_WINDOW_WINDOW].GetInt();
            window = to_string(temp);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_filter_window(string &window)
{
    bool goon = true;

    int window_tep = atoi(window.c_str());

    if(window_tep<0)
    {
        goon = false;
        window = string("put_filter_window: window must > 0");
    }

    if(window_tep>10)
    {
        goon = false;
        goon = false;
        window = string("put_filter_window: window must <= 10");
    }

    if(goon==true)
    {
        window = to_string(window_tep);

        string cmdback = put_async_http(FILTER_WINDOW_CMD, window);
        int res = cmdback.find_first_of(FILTER_WINDOW_ENSURE);
        res = strcmp(cmdback.c_str(), FILTER_WINDOW_ENSURE);
        if(!(res>=0))
        {
            goon = false;
            window = string("put_filter_window: oepration failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_filter_max_angle(string &max_angle)
{
    bool goon = true;
    string cmdback = get_async_http(FILTER_MAX_ANGLE_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(FILTER_MAX_ANGLE_MAX_ANGLE))
        {
            int temp = document[FILTER_MAX_ANGLE_MAX_ANGLE].GetInt();
            max_angle = to_string(temp);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_filter_max_angle(string &max_angle)
{
    bool goon = true;

    int max_angle_tep = atoi(max_angle.c_str());

    if(max_angle_tep<0)
    {
        goon = false;
        max_angle = string("put_filter_max_angle: max_angle must >= 0");
    }

    if(max_angle_tep>359)
    {
        goon = false;
        max_angle = string("put_filter_max_angle: max_angle must <= 359");
    }

    if(goon==true)
    {
        string string_min_angle_tep;
        get_filter_min_angle(string_min_angle_tep);
        int min_angle_tep = atoi(string_min_angle_tep.c_str());
        if(min_angle_tep>max_angle_tep)
        {
            goon = false;
            max_angle = string("put_filter_max_angle: max_angle must >= min_angle");
        }
    }

    if(goon==true)
    {
        string cmdback = put_async_http(FILTER_MAX_ANGLE_CMD, max_angle);
        int res = cmdback.find_first_of(FILTER_MAX_ANGLE_ENSURE);
        res = strcmp(cmdback.c_str(), FILTER_MAX_ANGLE_ENSURE);

        if(!(res>=0))
        {
            goon = false;
            max_angle = string("put_filter_max_angle: operation failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_filter_min_angle(string &min_angle)
{
    bool goon = true;
    string cmdback = get_async_http(FILTER_MIN_ANGLE_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);
    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(FILTER_MIN_ANGLE_MIN_ANGLE))
        {
            int temp = document[FILTER_MIN_ANGLE_MIN_ANGLE].GetInt();
            min_angle = to_string(temp);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_filter_min_angle(string &min_angle)
{
    bool goon = true;

    int min_angle_tep = atoi(min_angle.c_str());

    if(min_angle_tep<0)
    {
        goon = false;
        min_angle = string("put_filter_max_angle: max_angle must >= 0");
    }

    if(min_angle_tep>359)
    {
        goon = false;
        min_angle = string("put_filter_max_angle: max_angle must <= 359");
    }

    if(goon==true)
    {
        string string_max_angle_tep;
        get_filter_max_angle(string_max_angle_tep);
        int max_angle_tep = atoi(string_max_angle_tep.c_str());
        if(min_angle_tep>max_angle_tep)
        {
            goon = false;
            min_angle = string("put_filter_max_angle: max_angle must >= min_angle");
        }
    }

    string cmdback = put_async_http(FILTER_MIN_ANGLE_CMD, min_angle);
    int res = cmdback.find_first_of(FILTER_MIN_ANGLE_ENSURE);
    res = strcmp(cmdback.c_str(), FILTER_MIN_ANGLE_ENSURE);

    if(!(res>=0))
    {
        goon = false;
        min_angle = string("put_filter_max_angle: operation failed");
    }

    return goon;
}

bool LakiBeamHTTP::get_filter_neighbors(string &neighbors)
{
    bool goon = true;
    string cmdback = get_async_http(FILTER_NEIGHBORS_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(FILTER_NEIGHBORS_NEIGHBORS))
        {
            int temp = document[FILTER_NEIGHBORS_NEIGHBORS].GetInt();
            neighbors = to_string(temp);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_filter_neighbors(string &neighbors)
{
    bool goon = true;

    int neighbors_tep = atoi(neighbors.c_str());

    if(neighbors_tep<0)
    {
        goon = false;
        neighbors = string("put_filter_neighbors: neighbors must > 0");
    }
    if(neighbors_tep>10)
    {
        goon = false;
        neighbors = string("put_filter_neighbors: neighbors must < 10");
    }

    if(goon==true)
    {
        neighbors = to_string(neighbors_tep);
        string cmdback = put_async_http(FILTER_NEIGHBORS_CMD, neighbors);
        int res = cmdback.find_first_of(FILTER_NEIGHBORS_ENSURE);
        res = strcmp(cmdback.c_str(), FILTER_NEIGHBORS_ENSURE);

        if(!(res>=0))
        {
            goon = false;
            neighbors = string("put_filter_neighbors: operation failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_host(host_c &host)
{
    bool goon = true;
    string cmdback = get_async_http(HOST_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(HOST_HOST))
        {
            const value_t& value    = document[HOST_HOST];
            host.ip                 = value[HOST_IP].GetString();
            host.port               = value[HOST_PORT].GetInt();
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_host_ip(string &ip)
{
    bool goon = true;
    string cmdback = get_async_http(HOST_IP_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }
    if(goon==true)
    {
        if(document.HasMember(HOST_IP))
        {
            ip = document[HOST_IP].GetString();
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_host_ip(string &ip)
{
    bool goon = true;

    int sieze = ip.size();

    //for(int i=0;i<sieze;i++)
    //{
    //    if(((ip[i]<'1')&&(ip[i]!='.'))||((ip[i]>'9')&&(ip[i]!='.')))
    //    {
    //        goon = false;
    //        ip = string("put_host_ip: not ipv4 format");
    //    }
    //}

    if(goon==true)
    {
        if(!(address_t::from_string(ip).is_v4()))
        {
            goon = false;
            ip = string("put_host_ip: not ipv4 format");
        }
    }

    if(goon==true)
    {
        string cmdback = put_async_http(HOST_IP_CMD, ip);
        int res = cmdback.find_first_of(HOST_IP_ENSURE);
        res = strcmp(cmdback.c_str(), HOST_IP_ENSURE);

        if(!(res>=0))
        {
            goon = false;
            ip = string("put_host_ip: operation failed");
        }
    }

    return goon;
}

bool LakiBeamHTTP::get_host_port(string &port)
{
    bool goon = true;
    string cmdback = get_async_http(HOST_PORT_CMD);

    int bufsize = cmdback.size();

    char json[bufsize+1];
    memcpy(json, cmdback.c_str(), bufsize);
    json[bufsize] = '\0';

    document_t document;
    document.Parse(json);

    if(!document.IsObject())
    {
        goon = false;
    }

    if(goon==true)
    {
        if(document.HasMember(HOST_PORT))
        {
            int temp;
            temp = document[HOST_PORT].GetInt();
            port = to_string(temp);
        }
        else
        {
            goon = false;
        }
    }

    return goon;
}

bool LakiBeamHTTP::put_host_port(string &ip)
{
    bool goon = true;

    string cmdback = put_async_http(HOST_PORT_CMD, ip);
    int res = cmdback.find_first_of(HOST_PORT_ENSURE);
    res = strcmp(cmdback.c_str(), HOST_PORT_ENSURE);

    if(!(res>=0))
    {
        goon = false;
        ip = string("put_host_port: operation failed");
    }

    return goon;
}

string LakiBeamHTTP::get_http(string target)
{
    io_context_t    iocontext;
    resolver_t      resolver{iocontext};
    tcp_socket_t        socket{iocontext};

    auto const result = resolver.resolve(web_ip, web_port);

    boost::asio::connect(socket, result.begin(), result.end());

    http_n::request<http_n::string_body> req{ http_n::verb::get, target, 11};
    req.set(http_n::field::host, web_ip);
    req.set(http_n::field::user_agent, BOOST_BEAST_VERSION_STRING);
    req.set(http_n::field::connection, "close");

    http_n::write(socket, req);

    boost::beast::flat_buffer buffer;
    dynamic_body_t tempres;
    http_n::read(socket, buffer, tempres);

    boost::system::error_code ec;
    socket.shutdown(tcp_t::socket::shutdown_both, ec);

    ostringstream output_flow;
    output_flow << tempres;
    string cutstring = output_flow.str();

    int start   = cutstring.find_first_of("{", 0);
    int end     = cutstring.find_last_of("}");

    if(!(end<start))
    {
        cutstring.erase(end+1);
        cutstring.erase(0, start);
    }

    return cutstring;
}

string LakiBeamHTTP::put_http(string target, string message)
{
    io_context_t        iocontext;
    resolver_t          resolver{iocontext};
    tcp_socket_t        socket{iocontext};

    const int message_size = message.size();
    char message_buffer[message_size+1];
    string cutstring;

    if(message_size>0)
    {
        memcpy(message_buffer, message.c_str(), message_size);
        message_buffer[message_size] = '\0';
    }

    auto const result = resolver.resolve(web_ip, web_port);

    boost::asio::connect(socket, result.begin(), result.end());

    http_n::request<http_n::string_body> req{ http_n::verb::put, target, 11};
    req.set(http_n::field::host, web_ip);
    req.set(http_n::field::content_length, QString::number(message_size).toStdString());
    req.set(http_n::field::content_type, "application/json");
    req.body().append(reinterpret_cast<const char*>(message_buffer));

    http_n::write(socket, req);

    boost::beast::flat_buffer buffer;
    dynamic_body_t tempres;
    http_n::read(socket, buffer, tempres);

    boost::system::error_code ec;
    socket.shutdown(tcp_t::socket::shutdown_both, ec);

    ostringstream output_flow;
    output_flow << tempres;
    cutstring = output_flow.str();

    return cutstring;
}

string LakiBeamHTTP::delete_http(string target)
{
    io_context_t    iocontext;
    resolver_t      resolver{iocontext};
    tcp_socket_t        socket{iocontext};

    auto const result = resolver.resolve(web_ip, web_port);

    boost::asio::connect(socket, result.begin(), result.end());

    http_n::request<http_n::string_body> req{ http_n::verb::delete_, target, 11};
    req.set(http_n::field::host, web_ip);
    req.set(http_n::field::user_agent, BOOST_BEAST_VERSION_STRING);
    req.set(http_n::field::connection, "close");

    http_n::write(socket, req);

    boost::beast::flat_buffer buffer;
    dynamic_body_t tempres;
    http_n::read(socket, buffer, tempres);

    boost::system::error_code ec;
    socket.shutdown(tcp_t::socket::shutdown_both, ec);

    ostringstream output_flow;
    output_flow << tempres;
    string cutstring = output_flow.str();

    return cutstring;
}

string LakiBeamHTTP::get_async_http(string target)
{
    bool            goon = false;
    service_t       io_service;
    resolver_t      resolver(io_service);
    tcp_socket_t    socket(io_service);
    query_t         query(web_ip.c_str(), "80");
    tcp_endpoint_t  local(address_t::from_string(local_ip), atoi(local_port.c_str()));

    boost::system::error_code error;
    socket.open(local.protocol());
    attribute_tcp_t option(true);
    socket.set_option(option, error);
    socket.bind(local);
    char tempread[1024];

    string cutstring;

    auto read_hander = [&](const boost::system::error_code &error, size_t bytes)
    {
        if(!error)
        {
            (void)bytes;

            cutstring = string(tempread);
            cutstring.append(string("\0"));
            goon = true;
        }
    };

    auto connect_handler = [&](const boost::system::error_code &error)
    {
        if(!error)
        {
            http_n::request<http_n::string_body> req{ http_n::verb::get, target, 11};
            req.set(http_n::field::host, web_ip);
            req.set(http_n::field::user_agent, BOOST_BEAST_VERSION_STRING);
            req.set(http_n::field::connection, "close");

            stringstream request;
            request << req;

            string sendstring = request.str();
            int size = sendstring.size();
            char sendbuffer[size];
            memcpy(sendbuffer, sendstring.c_str(), size);

            boost::asio::write(socket, boost::asio::buffer(sendbuffer, size));

            socket.async_read_some(boost::asio::buffer(tempread, 1024), read_hander);
        }
    };

    auto resolve_handler = [&](const boost::system::error_code &error, iterator_t it)
    {
        if(!error)
        {
            socket.async_connect(*it, connect_handler);
        }
    };

    resolver.async_resolve(query, resolve_handler);

    io_service.run();

    int time = 0;

    while(goon==false)
    {
        boost::this_thread::sleep(boost::posix_time::microseconds(1));
        if(time++>HTTP_OVER_TIME)
        {
            break;
        }
    }

    boost::system::error_code ec;
    socket.shutdown(tcp_t::socket::shutdown_both, ec);

    int start   = cutstring.find_first_of("{", 0);
    int end     = cutstring.find_last_of("}");

    if(!(end<start))
    {
        cutstring.erase(end+1);
        cutstring.erase(0, start);
    }

    if(goon==false)
    {
        cutstring = string("get_async_http: cant't cnonnect to http");
    }

    return cutstring;
}

string LakiBeamHTTP::put_async_http(string target, string message)
{
    bool            goon = false;
    service_t       io_service;
    resolver_t      resolver(io_service);
    tcp_socket_t    socket(io_service);
    query_t         query(web_ip.c_str(), "80");
    tcp_endpoint_t  local(address_t::from_string(local_ip), atoi(local_port.c_str()));

    boost::system::error_code error;
    socket.open(local.protocol());
    attribute_tcp_t option(true);
    socket.set_option(option, error);
    socket.bind(local);
    char tempread[1024];

    string cutstring;

    auto read_hander = [&](const boost::system::error_code &error, size_t bytes)
    {
        if(!error)
        {
            (void)bytes;

            cutstring = string(tempread);
            cutstring.append(string("\0"));
            goon = true;
        }
    };

    auto connect_handler = [&](const boost::system::error_code &error)
    {
        if(!error)
        {
            const int message_size = message.size();
            char message_buffer[message_size+1];
            string cutstring;

            if(message_size>0)
            {
                memcpy(message_buffer, message.c_str(), message_size);
                message_buffer[message_size] = '\0';
            }

            http_n::request<http_n::string_body> req{ http_n::verb::put, target, 11};
            req.set(http_n::field::host, web_ip);
            req.set(http_n::field::content_length, QString::number(message_size).toStdString());
            req.set(http_n::field::content_type, "application/json");
            req.body().append(reinterpret_cast<const char*>(message_buffer));

            stringstream request;
            request << req << endl;

            string sendstring = request.str();
            int size = sendstring.size();
            char sendbuffer[size+1];
            memcpy(sendbuffer, sendstring.c_str(), size);
            sendbuffer[size] = '\0';

            boost::asio::write(socket, boost::asio::buffer(sendbuffer, size));

            socket.async_read_some(boost::asio::buffer(tempread), read_hander);
        }
    };

    auto resolve_handler = [&](const boost::system::error_code &error, iterator_t it)
    {
        if(!error)
        {
            socket.async_connect(*it, connect_handler);
        }
    };

    resolver.async_resolve(query, resolve_handler);

    io_service.run();

    int time = 0;

    while(goon==false)
    {
        boost::this_thread::sleep(boost::posix_time::microseconds(1));
        if(time++>HTTP_OVER_TIME)
        {
            break;
        }
    }

    boost::system::error_code ec;
    socket.shutdown(tcp_t::socket::shutdown_both, ec);

    if(goon==false)
    {
        cutstring = string("get_async_http: cant't cnonnect to http");
    }

    return cutstring;
}

string LakiBeamHTTP::delete_async_http(string target)
{
    bool            goon = false;
    service_t       io_service;
    resolver_t      resolver(io_service);
    tcp_socket_t    socket(io_service);
    query_t         query(web_ip.c_str(), "80");
    tcp_endpoint_t  local(address_t::from_string(local_ip), atoi(local_port.c_str()));

    boost::system::error_code error;
    socket.open(local.protocol());
    attribute_tcp_t option(true);
    socket.set_option(option, error);
    socket.bind(local);
    char tempread[1024];

    string cutstring;

    auto read_hander = [&](const boost::system::error_code &error, size_t bytes)
    {
        if(!error)
        {
            (void)bytes;

            cutstring = string(tempread);
            cutstring.append(string("\0"));
            goon = true;
        }
    };

    auto connect_handler = [&](const boost::system::error_code &error)
    {
        if(!error)
        {
            http_n::request<http_n::string_body> req{ http_n::verb::delete_, target, 11};
            req.set(http_n::field::host, web_ip);
            req.set(http_n::field::user_agent, BOOST_BEAST_VERSION_STRING);
            req.set(http_n::field::connection, "close");

            stringstream request;
            request << req << endl;

            string sendstring = request.str();
            int size = sendstring.size();
            char sendbuffer[size+1];
            memcpy(sendbuffer, sendstring.c_str(), size);
            sendbuffer[size] = '\0';

            boost::asio::write(socket, boost::asio::buffer(sendbuffer, size));

            socket.async_read_some(boost::asio::buffer(tempread), read_hander);
        }
    };

    auto resolve_handler = [&](const boost::system::error_code &error, iterator_t it)
    {
        if(!error)
        {
            socket.async_connect(*it, connect_handler);
        }
    };

    resolver.async_resolve(query, resolve_handler);

    io_service.run();

    int time = 0;

    while(goon==false)
    {
        boost::this_thread::sleep(boost::posix_time::microseconds(1));
        if(time++>HTTP_OVER_TIME)
        {
            break;
        }
    }

    boost::system::error_code ec;
    socket.shutdown(tcp_t::socket::shutdown_both, ec);

    if(goon==false)
    {
        cutstring = string("get_async_http: cant't cnonnect to http");
    }

    return cutstring;
}

bool LakiBeamHTTP::check_ipconfig()
{
    bool goon = true;

    if(!(address_t::from_string(local_ip).is_v4()))
    {
        goon = false;
        local_ip = string(HOST_IP_DEFAULT);
    }

    if(!address_t::from_string(web_ip).is_v4())
    {
        goon = false;
        web_ip = string(HTTP_IP_DEFAULT);
    }

    return goon;
}
