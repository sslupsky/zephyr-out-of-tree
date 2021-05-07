/**
 * @file witap_strings.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2019-11-14
 * 
 * @copyright Copyright (c) 2019
 * 
   _____                 _                _        _          
  / ____|               (_)              | |      (_)         
 | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
  \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
  ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
                                                              
 */


#ifndef WITAP_STRINGS_H
#define WITAP_STRINGS_H

#include <soc.h>

/**
 * @brief Text to ASCII Figlet Art
 *        Raw string literals:  https://stackoverflow.com/questions/1135841/c-multiline-string-literal
 */
char const STRING_SCAN_LOGO[]  = R"SCANLOGO(
   _____                 _                _        _          
  / ____|               (_)              | |      (_)         
 | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
  \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
  ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
                                                              
)SCANLOGO";

char const STRING_WITAP_LOGO[]  = R"WITAPLOGO(
 __        ___ _____  _    ____  TM
 \ \      / (_)_   _|/ \  |  _ \
  \ \ /\ / /| | | | / _ \ | |_) |
   \ V  V / | | | |/ ___ \|  __/ 
    \_/\_/  |_| |_/_/   \_\_|    
                                 
)WITAPLOGO";

char const STRING_SPACE[]  = " ";
char const STRING_NEWLINE[] = "\n";
char const STRING_HEADER[]  = "\n\n\n\n";
char const STRING_FORM_FEED[] = "\n\n\n\n\n\n\n\n\n\n";
char const STRING_SYNTAX_ERROR[]  = "Syntax error";

char const STRING_RCAUSE_POR[] =	"POR";
char const STRING_RCAUSE_BODCORE[] =	"BODCORE";
char const STRING_RCAUSE_BODVDD[] =	"BODVDD";
char const STRING_RCAUSE_NVM[] =	"NVM";
char const STRING_RCAUSE_EXT[] =	"EXT";
char const STRING_RCAUSE_WDT[] =	"WDT";
char const STRING_RCAUSE_SYST[] =	"SYST";
char const STRING_RCAUSE_BACKUP[] =	"BACKUP";

#ifdef MCLK
#define RCAUSE_POR	RSTC_RCAUSE_POR
#define RCAUSE_BODCORE	RSTC_RCAUSE_BODCORE
#define RCAUSE_BODVDD	RSTC_RCAUSE_BODVDD
#define RCAUSE_NVM	RSTC_RCAUSE_NVM
#define RCAUSE_EXT	RSTC_RCAUSE_EXT
#define RCAUSE_WDT	RSTC_RCAUSE_WDT
#define RCAUSE_SYST	RSTC_RCAUSE_SYST
#define RCAUSE_BACKUP	RSTC_RCAUSE_BACKUP
#else
#define RCAUSE_POR	PM_RCAUSE_POR
#define RCAUSE_BODCORE	PM_RCAUSE_BOD12
#define RCAUSE_BODVDD	PM_RCAUSE_BOD33
#define RCAUSE_NVM	0
#define RCAUSE_EXT	PM_RCAUSE_EXT
#define RCAUSE_WDT	PM_RCAUSE_WDT
#define RCAUSE_SYST	PM_RCAUSE_SYST
#define RCAUSE_BACKUP	0
#endif

#ifndef RSTC
#define RSTC PM
#endif

inline uint8_t parse_reset(char *buf)
{
	uint8_t rcause = RSTC->RCAUSE.reg;

	if (buf) {
		buf[0] = 0;
		if (rcause & RCAUSE_POR) {
			strcat(buf, STRING_SPACE);
			strcat(buf, STRING_RCAUSE_POR);
		}
		if (rcause & RCAUSE_BODCORE) {
			strcat(buf, STRING_SPACE);
			strcat(buf, STRING_RCAUSE_BODCORE);
		}
		if (rcause & RCAUSE_BODVDD) {
			strcat(buf, STRING_SPACE);
			strcat(buf, STRING_RCAUSE_BODVDD);
		}
		if (rcause & RCAUSE_NVM) {
			strcat(buf, STRING_SPACE);
			strcat(buf, STRING_RCAUSE_NVM);
		}
		if (rcause & RCAUSE_EXT) {
			strcat(buf, STRING_SPACE);
			strcat(buf, STRING_RCAUSE_EXT);
		}
		if (rcause & RCAUSE_WDT) {
			strcat(buf, STRING_SPACE);
			strcat(buf, STRING_RCAUSE_WDT);
		}
		if (rcause & RCAUSE_SYST) {
			strcat(buf, STRING_SPACE);
			strcat(buf, STRING_RCAUSE_SYST);
		}
		if (rcause & RCAUSE_BACKUP) {
			strcat(buf, STRING_SPACE);
			strcat(buf, STRING_RCAUSE_BACKUP);
		}
	}

	return rcause;
}

#endif
