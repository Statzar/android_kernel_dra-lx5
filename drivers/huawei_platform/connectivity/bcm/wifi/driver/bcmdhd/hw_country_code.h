#ifndef __HW_COUNTRY_CODE_H__
#define __HW_COUNTRY_CODE_H__

struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int32 custom_locale_rev;
};

/* Customized Locale table : OPTIONAL feature */
const struct cntry_locales_custom hw_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
    /* Table should be filled out based
    on custom platform regulatory requirement */
    /* default ccode/regrev */
    {"",   "XZ", 11},   /* Universal if Country code is unknown or empty */
    {"IR", "XZ", 11}, 	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
    {"SD", "XZ", 11}, 	/* Universal if Country code is SUDAN */
    {"SY", "XZ", 11}, 	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
    {"GL", "XZ", 11}, 	/* Universal if Country code is GREENLAND */
    {"PS", "XZ", 11}, 	/* Universal if Country code is PALESTINIAN TERRITORY, OCCUPIED */
    {"TL", "XZ", 11}, 	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
    {"MH", "XZ", 11}, 	/* Universal if Country code is MARSHALL ISLANDS */
    {"CU", "CN", 38},
    {"KP", "CN", 38},
    {"BD", "CN", 38},   /* Hw CN if Country code is Bangladesh*/
    {"UA", "CN", 38},	/* Hw CN if Country code is Ukraine*/
    {"AO", "CN", 38},   /* Hw CN if Country code is Angola*/
    {"AL", "AL", 2},
    {"DZ", "GB", 6},
    {"AS", "AS", 12},
    {"AI", "AI", 1},
    {"AF", "AD", 0},
    {"AG", "AG", 2},
    {"AR", "AU", 6},
    {"AW", "AW", 2},
    {"AU", "AU", 6},
    {"AT", "AT", 4},
    {"AZ", "AZ", 2},
    {"BS", "BS", 2},
    {"BH", "BH", 4},
    {"BY", "BY", 3},
    {"BE", "BE", 4},
    {"BM", "BM", 12},
    {"BA", "BA", 2},
    {"BR", "BR", 4},
    {"VG", "VG", 2},
    {"BN", "BN", 4},
    {"BG", "BG", 4},
    {"KH", "KH", 2},
    {"CA", "CA", 2},
    {"KY", "KY", 3},
    {"CN", "CN", 38},
    {"CR", "CR", 17},
    {"HR", "HR", 4},
    {"CY", "CY", 4},
    {"CZ", "CZ", 4},
    {"DK", "DK", 4},
    {"EE", "EE", 4},
    {"ET", "ET", 2},
    {"FI", "FI", 4},
    {"FR", "FR", 5},
    {"GF", "GF", 2},
    {"DE", "DE", 7},
    {"GR", "GR", 4},
    {"GD", "GD", 2},
    {"GP", "GP", 2},
    {"GU", "GU", 12},
    {"HK", "HK", 2},
    {"HU", "HU", 4},
    {"IS", "IS", 4},
    {"IN", "IN", 3},
    {"ID", "ID", 1},
    {"IE", "IE", 5},
    {"IL", "IL", 7},
    {"IT", "IT", 4},
    {"JP", "JP", 58},
    {"JO", "JO", 3},
    {"KE", "SA", 0},
    {"KW", "KW", 5},
    {"LA", "LA", 2},
    {"LV", "LV", 4},
    {"LS", "LS", 2},
    {"LI", "LI", 4},
    {"LT", "LT", 4},
    {"LU", "LU", 3},
    {"MK", "MK", 2},
    {"MW", "MW", 1},
    {"MY", "MY", 19},
    {"MV", "MV", 3},
    {"MT", "MT", 4},
    {"MQ", "MQ", 2},
    {"MR", "MR", 2},
    {"MU", "MU", 2},
    {"YT", "YT", 2},
    {"MX", "HK", 2},
    {"MD", "MD", 2},
    {"MC", "MC", 1},
    {"ME", "ME", 2},
    {"MA", "MA", 2},
    {"NP", "ID", 5},
    {"NL", "NL", 4},
    {"AN", "GD", 2},
    {"NZ", "NZ", 4},
    {"NO", "NO", 4},
    {"OM", "OM", 4},
    {"PA", "PA", 17},
    {"PG", "AU", 6},
    {"PY", "PY", 2},
    {"PE", "PE", 20},
    {"PH", "PH", 5},
    {"PL", "PL", 4},
    {"PT", "PT", 4},
    {"RE", "RE", 2},
    {"RO", "RO", 4},
    {"SN", "MA", 2},
    {"RS", "RS", 2},
    {"SK", "SK", 4},
    {"SI", "SI", 4},
    {"ES", "ES", 4},
    {"LK", "LK", 1},
    {"SE", "SE", 4},
    {"CH", "CH", 4},
    {"TW", "TW", 65},
    {"TH", "TH", 5},
    {"TT", "TT", 3},
    {"TR", "TR", 7},
    {"AE", "AE", 6},
    {"UG", "TW", 1},
    {"GB", "GB", 6},
    {"UY", "UY", 0},
    {"VI", "VI", 13},
    {"VA", "VA", 2},
    {"VE", "VE", 3},
    {"VN", "VN", 4},
    {"ZM", "LA", 2},
    {"EC", "EC", 21},
    {"SV", "SV", 25},
    {"KR", "KR", 48},
    {"RU", "RU", 13},
    {"GT", "GT", 1},
    {"FR", "FR", 5},
    {"MN", "MN", 1},
    {"NI", "NI", 2},
    {"UZ", "MA", 2},
};

#endif
