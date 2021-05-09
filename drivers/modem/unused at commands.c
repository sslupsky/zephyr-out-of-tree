/**
 * @file unused at commands.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-04
 * 
 * @copyright Copyright (c) 2020
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
 */


/* AT+REBOOT Handler: +OK[0]=<REBOOT>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_reboot)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("DEVEUI: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+BAND Handler: +OK[0]=<BAND>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_band)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_band, sizeof(minfo.mdm_band) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_band[out_len] = '\0';
	LOG_INF("BAND: %s", log_strdup(minfo.mdm_band));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+ADR Handler: +OK[0]=<ADR>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_adr)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_adr, sizeof(minfo.mdm_adr) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_adr[out_len] = '\0';
	LOG_INF("ADR: %s", log_strdup(minfo.mdm_adr));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+RFPOWER Handler: +OK[0]=<RFPOWER>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_rfpower)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_txpower, sizeof(minfo.mdm_txpower) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_txpower[out_len] = '\0';
	LOG_INF("RFPOWER: %s", log_strdup(minfo.mdm_txpower));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+DFORMAT Handler: +OK[0]=<DFORMAT>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_dformat)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_atformat, sizeof(minfo.mdm_atformat) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_atformat[out_len] = '\0';
	LOG_INF("DFORMAT: %s", log_strdup(minfo.mdm_atformat));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+DR Handler: +OK[0]=<DR>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_dr)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_dr, sizeof(minfo.mdm_dr) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_dr[out_len] = '\0';
	LOG_INF("DR: %s", log_strdup(minfo.mdm_dr));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+DUTYCYCLE Handler: +OK[0]=<DUTYCYCLE>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_dutycycle)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_dutycycle, sizeof(minfo.mdm_dutycycle) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_dutycycle[out_len] = '\0';
	LOG_INF("DUTYCYCLE: %s", log_strdup(minfo.mdm_dutycycle));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+NWK Handler: +OK[0]=<NWK>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_nwk)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("NWK: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+MODE Handler: +OK[0]=<MODE>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_mode)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_mode, sizeof(minfo.mdm_mode) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_mode[out_len] = '\0';
	LOG_INF("MODE: %s", log_strdup(minfo.mdm_mode));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+FCU Handler: +OK[0]=<FCU>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_fcu)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_fcu, sizeof(minfo.mdm_fcu) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_fcu[out_len] = '\0';
	LOG_INF("FCU: %s", log_strdup(minfo.mdm_fcu));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+FCD Handler: +OK[0]=<FCD>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_fcd)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_fcd, sizeof(minfo.mdm_fcd) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_fcd[out_len] = '\0';
	LOG_INF("FCD: %s", log_strdup(minfo.mdm_fcd));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+CLASS Handler: +OK[0]=<CLASS>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_class)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_class, sizeof(minfo.mdm_class) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_class[out_len] = '\0';
	LOG_INF("CLASS: %s", log_strdup(minfo.mdm_class));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+JOIN Handler: +OK[0]=<JOIN>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_join)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("JOIN: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+NJS Handler: +OK[0]=<NJS>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_njs)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_njs, sizeof(minfo.mdm_njs) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_njs[out_len] = '\0';
	LOG_INF("NJS: %s", log_strdup(minfo.mdm_njs));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+UTX Handler: +OK[0]=<UTX>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_utx)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("UTX: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+CTX Handler: +OK[0]=<CTX>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_ctx)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("CTX: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+PORT Handler: +OK[0]=<PORT>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_port)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_port, sizeof(minfo.mdm_port) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_port[out_len] = '\0';
	LOG_INF("PORT: %s", log_strdup(minfo.mdm_port));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+CFM Handler: +OK[0]=<CFM>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_cfm)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_cfm, sizeof(minfo.mdm_cfm) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_cfm[out_len] = '\0';
	LOG_INF("CFM: %s", log_strdup(minfo.mdm_cfm));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+CFS Handler: +OK[0]=<CFS>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_cfs)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_cf, sizeof(minfo.mdm_cf) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_cf[out_len] = '\0';
	LOG_INF("CFS: %s", log_strdup(minfo.mdm_cf));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+SNR Handler: +OK[0]=<SNR>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_snr)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_snr, sizeof(minfo.mdm_snr) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_snr[out_len] = '\0';
	LOG_INF("SNR: %s", log_strdup(minfo.mdm_snr));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+RSSI Handler: +OK[0]=<RSSI>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_rssi)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_rssi, sizeof(minfo.mdm_rssi) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_rssi[out_len] = '\0';
	LOG_INF("RSSI: %s", log_strdup(minfo.mdm_rssi));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+BAT Handler: +OK[0]=<BAT>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_bat)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_bat, sizeof(minfo.mdm_bat) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_bat[out_len] = '\0';
	LOG_INF("BAT: %s", log_strdup(minfo.mdm_bat));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+UART Handler: +OK[0]=<UART>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_uart)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("UART: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+FACNEW Handler: +OK[0]=<FACNEW>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_facnew)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("FACNEW: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+SLEEP Handler: +OK[0]=<SLEEP>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_sleep)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("SLEEP: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+MSIZE Handler: +OK[0]=<MSIZE>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmd_msize)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("MSIZE: %s", log_strdup(minfo.mdm_deveui));
	k_sem_give(&lora.sem_response);

	return 0;
}
