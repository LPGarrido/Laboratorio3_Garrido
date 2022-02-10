;Archivo:	main_laboratorio3.s
;Dispositivo:	PIC16F887
;Autor:		Luis Garrido
;Compilador:	pic-as (v2.30), MPLABX V5.40
;
;Programa:	Contador de 4 bits en el puerto B y D, Bandera en el RE0,  Display hexadecimal en el puerto C
;Hardware:	Leds en el puerto B y D, Led en el RE0, Display en el puerto C
;Creado: 06 feb, 2022 
;Última modificación: 10 feb, 2022

PROCESSOR 16F887
    
; PIC16F887 Configuration Bit Settings
; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = OFF              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

// config statements should precede project file includes.
#include <xc.inc>

PSECT udata_bank0	; Reservar espacio de memoria
  CONT:	    DS	    1	; Variable CONT 1 byte
  CONT2:    DS	    1	; Variable CONT2 1 byte
  COMP:	    DS	    1	; Variable COMP 1 byte
  
PSECT resVect, class=CODE, abs, delta=2
ORG 00h	    ; posición 00h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL MAIN	; Cambio de pagina
    GOTO    MAIN	; Ir al main
    
PSECT code, delta=2, abs
ORG 100h    ; posición 100h para el codigo
;------------- TABLA ------------
TABLA:
    CLRF    PCLATH	; Limpiar PCLATH
    BSF	    PCLATH,0	; PCLATH = 01	PC = 02
    ANDLW   0x0F	; Solo permitir valores iguales o menores a 0x0F
    ADDWF   PCL,F	; PC = PCLATH + PCL + W
    ;HGFEDCBA		
    RETLW   00111111B	; 0
    RETLW   00000110B	; 1
    RETLW   01011011B	; 2
    RETLW   01001111B	; 3
    RETLW   01100110B	; 4
    RETLW   01101101B	; 5
    RETLW   01111101B	; 6
    RETLW   00000111B	; 7
    RETLW   01111111B	; 8
    RETLW   01100111B	; 9
    RETLW   01110111B	; A
    RETLW   01111100B	; B
    RETLW   00111001B	; C
    RETLW   01011110B	; D
    RETLW   01111001B	; E
    RETLW   01110001B	; F
    
;------------- CONFIGURACION ------------
MAIN:
    CALL    CONFIG_IO	    ; Configuración de I/O
    CALL    CONFIG_RELOJ    ; Configuración de Oscilador
    CALL    CONFIG_TMR0	    ; Configuración de TMR0
    BANKSEL PORTD	    ; Cambio de banco

;-------LOOP PRINCIPAL------------------    
    
LOOP:
    BTFSC   T0IF	    ; T0IF = 0, Evaluar ; T0IF=1, CALL FUNC_TMR0
    CALL    FUNC_TMR0
    BTFSS   PORTA,0	    ; RA0 = 0,CALL INC_CONT; RA0=1, Evaluar
    CALL    INC_CONT
    BTFSS   PORTA,1	    ; RA1 = 0,CALL DEC_CONT; RA1=1, Evaluar
    CALL    DEC_CONT
    CALL    COMPARADOR	    ; Comparar la cuenta del display y contador de seg
    GOTO    LOOP	    ; regresa

;-------SUBRUTINAS------------------    
    
CONFIG_IO:
    BANKSEL ANSEL	    ; Cambio de banco
    CLRF    ANSEL	    ; I/O digitales
    CLRF    ANSELH	    ; I/O digitales
    BANKSEL TRISD	    ; Cambio de banco
    MOVLW   0x7F	    ; Pasar el num a W 
    MOVWF   TRISA	    ; Pasar W al TRISA (RA0,RA1 como Entradas)
    MOVLW   0xF0	    ; Pasar el num a W 
    MOVWF   TRISD	    ; Pasar W al TRISD (RD0,RD1,RD2,RD3 como Salida)
    CLRF    TRISC	    ; Limpiar TRISC (PORTC como Salidas)
    MOVLW   0xF0	    ; Pasar el num a W 
    MOVWF   TRISB	    ; Pasar W al TRISD (RD0,RD1,RD2,RD3 como Salida)
    BCF	    TRISE,0	    ; RE0 como Salida
    BANKSEL PORTD	    ; Cambio de banco
    CLRF    PORTD	    ; Limpiar PORTD
    CLRF    PORTC	    ; Limpiar PORTC
    CLRF    PORTB	    ; Limpiar PORTB
    CLRF    PORTE	    ; Limpiar PORTE
    ;MOVWF   0x01	    ; Limpiar W
    RETURN		    ; Regresar
    
CONFIG_RELOJ:
    BANKSEL OSCCON	    ; Cambio de banco
    BSF	    SCS		    ; SCS = 1, Usamos reloj interno
    BSF	    IRCF2	    ; IRCF2 = 1
    BCF	    IRCF1	    ; IRCF1 = 0
    BSF	    IRCF0	    ; IRCF0 = 1 - IRCF<2:0> => 101 2MHz
    RETURN		    ; Regresar
    
; Configuramos el TMR0 para obtener un retardo de 100ms
CONFIG_TMR0:
    BANKSEL OPTION_REG	    ; Cambio de banco
    BCF	    T0CS	    ; TMR0 = 0, Timer0 como temporizador
    BCF	    PSA		    ; Prescaler a TMR0
    BSF	    PS2		    ; PS2 = 1
    BSF	    PS1		    ; PS1 = 1
    BSF	    PS0		    ; PS0 = 1 - PS<2:0> => 111 prescaler 1 : 256
    
    BANKSEL TMR0	    ; Cambio de banco
    MOVLW   61		    ; Valor a cargar para tener un retardo de 100ms
    MOVWF   TMR0	    ; 100ms retardo
    BCF	    T0IF	    ; Limpiar bandera de interrupción
    RETURN		    ; Regresa

; Cada vez que se cumple el tiempo del TMR0 es necesario reiniciarlo.
RESET_TMR0:
    BANKSEL TMR0	    ; Cambio de banco
    MOVLW   61		    ; Valor a cargar para tener un retardo de 100ms
    MOVWF   TMR0	    ; 100ms retardo
    BCF	    T0IF	    ; Limpiar bandera de interrupción
    
    INCF CONT2,F	    ; Incrementar CONT2 cada vez que T01F se enciende
    MOVLW 10		    ; W = 10
    SUBWF CONT2,W	    ; W = 10-CONT2, Cuando CONT2=10, el bit STATUS,2 se enciende
    BTFSC STATUS,2	    ; Si Z=1, sume al portb; Z=0 return
    CALL INC_PORTB	    ; Incrementa el PortB cada segundo
    RETURN		    ; Regresa
    
INC_PORTB:
    INCF PORTB		    ; Incrementar el port B
    CLRF CONT2		    ; Limpiar la variable CONT2
    RETURN		    ; Regresar
    
COMPARADOR:
    MOVF    CONT,W	    ; W = CONT
    ANDLW   0x0F	    ; W = CONT valores menores a 0x0F
    MOVWF   COMP	    ; COMP = CONT valores menores a 0x0F
    MOVF    PORTB,W	    ; W = PORTB
    ANDLW   0x0F	    ; W = PORTB valores menores a 0x0F
    XORWF   COMP,F	    ; XOR entre CONT y PORTB con valores menores a 0x0F
    BTFSC   STATUS,2	    ; Z=1, CAMBIO; Z=0, RETURN 
    CALL    CAMBIO	    ; CAMBIO del bit RE0
    RETURN
    
CAMBIO:
    CLRF   PORTB	    ; Limipiar el PORTB
    INCF   PORTE	    ; Si RE0=0 pasa a ser 1 y si RE0=1 pasa a ser 0
    RETURN		    ; Regresa
    
    
INC_CONT:
    BTFSS PORTA,0   ; RA0=0 - GOTO INC_CONT, RA0=1 - INCREMENTAR
    GOTO INC_CONT
    INCF CONT,F     ; CONT SE INCREMENTA 1
    MOVF CONT,W	    ; W = CONT
    CALL TABLA	    ; Llama a la TABLA
    MOVWF PORTC	    ; Regresa con un valor en W que lo pasamos al puerto C
    RETURN	    ; RETURN
    
DEC_CONT:
    BTFSS PORTA,1   ;RA1=0 - GOTO CONT, RA1=1 - DECF CONT,F
    GOTO DEC_CONT   ;REGRESA
    DECF CONT,F     ;CONT SE DECREMENTA 1
    MOVF CONT,W	    ; W = CONT
    CALL TABLA	    ; Llama a la TABLA
    MOVWF PORTC	    ; Regresa con un valor en W que lo pasamos al puerto C
    RETURN	    ; RETURN
    
FUNC_TMR0:
    ; Cuando T0IF = 1, se resetea el TMR0 y ejecuta las instrucciones.
    CALL    RESET_TMR0	    ; Resetear el TMR0
    INCF    PORTD,F	    ; Incrementar en 1 el PORTD
    RETURN		    ; Regresa 
    
END


