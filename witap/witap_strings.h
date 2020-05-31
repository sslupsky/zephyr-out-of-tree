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

#define RCAUSE_POR	"POR"
#define RCAUSE_BOD12	"BOD12"
#define RCAUSE_BOD33	"BOD33"
#define RCAUSE_EXT	"EXT"
#define RCAUSE_WDT	"WDT"
#define RCAUSE_SYST	"SYST"

inline void rcause_str(char *buf, u8_t rcause)
{
	buf[0] = 0;
	if (rcause & PM_RCAUSE_POR) {
		strcat(buf, "POR ");
	}
	if (rcause & PM_RCAUSE_BOD12) {
		strcat(buf, "BOD12 ");
	}
	if (rcause & PM_RCAUSE_BOD33) {
		strcat(buf, "BOD33 ");
	}
	if (rcause & PM_RCAUSE_EXT) {
		strcat(buf, "EXT ");
	}
	if (rcause & PM_RCAUSE_WDT) {
		strcat(buf, "WDT ");
	}
	if (rcause & PM_RCAUSE_SYST) {
		strcat(buf, "SYST ");
	}
}

#endif
