/**
 ******************************************************************************
 * @file    http_parse.h
 * @author  QQ DING
 * @version V1.0.0
 * @date    1-September-2015
 * @brief   This maintains the functions that are commonly used by both the
 *          HTTP client and the HTTP server for parsing the HTTP requests.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2014 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */

#ifndef __HTTP_PARSE_H__
#define __HTTP_PARSE_H__

#include "httpd.h"


/** Parse tag/value form elements present in HTTP POST body
 *
 * Given a tag this function will retrieve its value from the buffer and return
 * it to the caller.
 * \param[in] inbuf Pointer to NULL-terminated buffer that holds POST data
 * \param[in] tag The tag to look for
 * \param[out] val Buffer where the value will be copied to
 * \param[in] val_len The length of the val buffer
 *
 *
 * \return WM_SUCCESS when a valid tag is found, error otherwise
 */
int httpd_get_tag_from_post_data(char *inbuf, const char *tag,
			  char *val, unsigned val_len);

/** Parse tag/value form elements present in HTTP GET URL
 *
 * Given a tag this function will retrieve its value from the HTTP URL and
 * return it to the caller.
 * \param[in] inbuf Pointer to NULL-terminated buffer that holds POST data
 * \param[in] tag The tag to look for
 * \param[out] val Buffer where the value will be copied to
 * \param[in] val_len The length of the val buffer
 *
 * \return WM_SUCCESS when a valid tag is found, error otherwise
 */
int httpd_get_tag_from_url(httpd_request_t *req_p,
			const char *tag,
			char *val, unsigned val_len);

int htsys_getln_soc(int sd, char *data_p, int buflen);

#endif /* __HTTP_PARSE_H__ */
