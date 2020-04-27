/*
 * string_manipulation.h
 *
 *  Created on: Apr 15, 2020
 *      Author: Itsuki Sakamoto
 */

#ifndef INC_STRING_MANIPULATION_H_
#define INC_STRING_MANIPULATION_H_

int wifi_credential_search(char * string, char * name, char * password, int size);
void manipulate_string(char * msg, int msg_size);
int get_temperature (char * msg, int msg_size);
void extract_time (char * msg, int msg_size, char * time);

#endif /* INC_STRING_MANIPULATION_H_ */
