/*****************************************************************************
*****              (C)2014 FLIR Commercial Systems, Inc.                 *****
*****                       All Rights Reserved.                         *****
*****                                                                    *****
*****     This source data and code (the "Code") may NOT be distributed  *****
*****     without the express prior written permission consent from of   *****
*****     FLIR Commercial Systems, Inc. ("FLIR").  FLIR PROVIDES THIS    *****
*****     CODE ON AN "AS IS" BASIS FOR USE BY RECIPIENT AT ITS OWN       *****
*****     RISK.  FLIR DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, IMPLIED *****
*****     OR STATUTORY, INCLUDING WITHOUT LIMITATION ANY IMPLIED         *****
*****     WARRANTIES OF TITLE, NON-INFRINGEMENT OF THIRD PARTY RIGHTS,   *****
*****     MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.          *****
*****     FLIR Commercial Systems, Inc. reserves the right to make       *****
*****     changes without further notice to the Code or any content      *****
*****     herein including to improve reliability, function or design.   *****
*****     FLIR Commercial Systems, Inc. shall not assume any liability   *****
*****     arising from the application or use of this code, data or      *****
*****     function.                                                      *****
*****                                                                    *****
*****     FLIR Commercial Systems, Inc.                                  *****
*****     Motion Control Systems                                         *****
*****     www.flir.com/mcs                                               *****
*****     mcs-support@flir.com                                           *****
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include "cpi.h"

typedef enum { ARG, RET } spec_dir;

static char gen_var_decl(char *spec, char name, spec_dir dir){
    while(*spec){
        printf("    ");
        switch(*spec){
        case CPI_T_UNSIGNED:
            printf("unsigned int");
            break;
        case CPI_T_DOUBLE:
            printf("double");
            break;
        case CPI_T_STR:
            printf("char%s", dir == ARG ? " *" : "");
            break;
        case CPI_T_BINARY:
            printf("unsigned char");
            break;
        case CPI_T_INT:
        case CPI_T_ENUM:
            printf("int");
            break;
        default:
            printf("[!] Unknown type: '%c'\n", *spec);
            exit(1);
        }
        printf(" %c", name);
        switch(*spec){
        case CPI_T_STR:
            if(dir == ARG){
                printf(" = \"input string\"");
            } else {
                printf("[MAX_LEN]");
            }
            break;
        case CPI_T_BINARY:
            printf("[MAX_LEN]");
            break;
        }
        printf(";\n");
        /* XXX: hack since only single binary arg/ret ops exist */
        if(*spec == CPI_T_BINARY && dir == RET){
            printf("    int count;\n");
        }
        name++;
        spec++;
    }

    return name;
}


static void gen_spec(char *spec, char name, spec_dir dir){
    while(*spec){
        switch(*spec){
        case CPI_T_STR:
            if(dir == ARG){
                printf(", %c", name);
            } else {
                printf(", MAX_LEN, NULL, %c", name);
            }
            break;
        case CPI_T_BINARY:
            if(dir == ARG){
                printf(", (int)sizeof(%c), %c", name, name);
            } else {
                printf(", MAX_LEN, &count, %c", name);
            }
            break;
        default:
            printf(", %s%c", dir == RET ? "&" : "", name);
        }
        name++;
        spec++;
    }
}


/* generate a simple C function with an opcode */
static int generate(const char *in_opname){
    cpi_opcode          op;
    char                *opname, *sp, ret_name, last_name;
    struct cpi_opent    *ent;

    /* duplicate and make upper-case the user's opname */
    opname = strdup(in_opname);
    sp = opname;
    while(*sp){ *sp = toupper(*sp); sp++; }

    /* look up the opcode */
    if((op = cpi_opname_search(opname)) == OP_INVALID){
        printf("[!] Opcode %s not found.\n", opname);
        return 1;
    }

    /* lookup entry */
    if((ent = cpi_op_lkup(op)) == NULL){
        printf("[!] Opcode %s has no entry.\n", opname);
        return 1;
    }

    /* generate code
     * variables are declared in left-right order with argspec first
     */
    printf( "int ptu_command(cerialh cer, uint16_t *status){\n"
            "    int rc;\n"
          );
    ret_name = gen_var_decl(ent->argspec, 'a', ARG);
    last_name = gen_var_decl(ent->retspec, ret_name, RET);
    printf("\n");
    if(ret_name > 'a'){
        printf( "    /* XXX: set %s's arguments */\n"
                "\n",
                opname);
    }
    printf("    rc = cpi_ptcmd(cer, &status, %s", opname);
    gen_spec(ent->argspec, 'a', ARG);
    gen_spec(ent->retspec, ret_name, RET);
    printf( ");\n\n");
    if(last_name > ret_name){
        printf( "    /* XXX: do something with %s's return values */\n"
                "\n",
                opname);
    }
    printf( "    return rc;\n"
            "}\n"
          );

    return 0;
}


static void print_help(void){
    printf( "Usage: opgen [arguments]\n"
            "Options:\n"
            " -h            - This help message\n"
            " -g <name>     - Generate code to for opcode <name>\n"
          );
}


int main(int argc, char *argv[]){
    int c, gens;

    gens = 0;
    while((c = getopt(argc, argv, "hg:")) != -1){
        switch(c){
        case 'h':
            print_help();
            exit(0);
            break;
        case 'g':
            if(generate(optarg)){
                exit(1);
            }
            gens++;
            break;
        default:
            printf("[!] Unknown option: %c\n", c);
            exit(1);
        }
    }

    if(gens == 0){
        printf("[!] Nothing to do.\n");
        return 1;
    } else {
        printf("\n");
        printf("[*] Done\n");
        return 0;
    }
}

