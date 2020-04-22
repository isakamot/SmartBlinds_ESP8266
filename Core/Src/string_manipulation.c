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

