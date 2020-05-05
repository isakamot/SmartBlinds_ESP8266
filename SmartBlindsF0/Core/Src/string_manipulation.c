#include "main.h"

int wifi_credential_search(char * string, char * name, char * password, int size){
    int i, j, k;
    int name_flag;
    int password_flag;
    int name_finish;
    int password_finish;

    name_finish = 0;
    password_flag = 0;
    name_flag = 0;
    password_finish = 0;

    i = 0;
    j = 0;
    k = 0;

	while (i != size){
	    if (string[i] == '"' && name_flag == 0 && name_finish == 0){
	        name_flag = 1;
	    }
	    else if (string[i] == '"' && name_flag == 1){
	        name_flag = 0;
	        name_finish = 1;
	        name[j] = string[i];
	    }
	    else if (string[i] == '"' && password_flag == 0 && name_finish == 1){
	        password_flag = 1;
	    }
	    else if (string[i] == '"' && password_flag == 1){
	        password_flag = 0;
	        password[k] = string[i];
	        password_finish = 1;
	    }
	    if (name_flag){
	        name[j] = string[i];
	        j++;
	    }
	    else if (password_flag){
	        password[k] = string[i];
	        k++;
	    }
	    i++;
	}

	if (password_finish && name_finish){
	    return 1;
	}
	return 0;
}

void manipulate_string(char * msg, int msg_size){
	int i;
	for (i=0; i < msg_size; i++){
		if (msg[i] == '\0'){
			msg[i] = '_';
		}
	}
}

int get_temperature (char * msg, int msg_size){
	int i;
	char temp_str[2] = "";
	int temp_int = 0;
	int temp_flag = 0;
	int temp_index = 0;

	for (i = 0; i < msg_size; i++){
	    if (msg[i] == '\r'){
			temp_flag = 0;
		}
	    if (temp_flag){
			temp_str[temp_index] = msg[i];
			temp_index ++;
		}
		if (msg[i] == '/'){
			temp_flag = 1;
		}
	}

	temp_int = atoi(temp_str);

	return temp_int;
}

void extract_time (char * msg, int msg_size, char * time){
	int i;
	int time_flag = 0;
	int time_index = 0;

	for (i = 0; i < msg_size; i++){
		if (time_flag && msg[i] == '\r'){
			time_flag = 0;
		}

		if (time_flag){
			time[time_index] = msg[i];
			time_index++;
		}

		if (msg[i] == '/'){
			time_flag = 1;
		}
	}

}

void get_time_struct (struct time * t, char * time_str, int PM_flag){
	char hour[3] = "";
	char minute[3] = "";

	hour[0] = time_str[0];
	hour[1] = time_str[1];
	minute[0] = time_str[3];
	minute[1] = time_str[4];

	t->hour = atoi(hour);
	t->minute = atoi(minute);
	if (PM_flag){
		t->hour+=12;
	}

}

