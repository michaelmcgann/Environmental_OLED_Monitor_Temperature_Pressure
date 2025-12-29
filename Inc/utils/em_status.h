

#ifndef UTILS_EM_STATUS_H_
#define UTILS_EM_STATUS_H_

typedef enum {
	EM_OK = 0,
	EM_E_PARAM,
	EM_E_TIMEOUT,
	EM_E_IO,
	EM_E_NOT_FOUND,
	EM_E_BUSY,
	EM_E_STATE,
	EM_E_NULL
} em_status_t;

#define EM_SUCCEEDED(s) ( ( s ) == EM_OK )
#define EM_FAILED(s)    ( ( s ) != EM_OK )


#endif /* UTILS_EM_STATUS_H_ */
