#ifndef __KEYBOARD_H
#define __KEYBOARD_H
#include "sys.h"
#include "Mydefine.h"


#define VIDEO PAout(8)	// ͼ����·�л�


void ButtonStatu_Verdict(KeyBoardTypeDef * Key,u8 key_value);
void Res_Deal(u8 key[8],u8 res);

void VideoRelay_Init(void);

void KeyG_Deal(u8 key_value);
void KeyF_Deal(u8 key_value);
void KeyB_Deal(u8 key_value);
void KeyV_Deal(u8 key_value);
void KeyE_Deal(u8 key_value);
void KeyR_Deal(u8 key_value);
void KeyC_Deal(u8 key_value);
void KeyQ_Deal(u8 key_value);	//�ǵ�ģʽ����

void MainBoard_Deal(void);
void TakeBullet(u8 input);

#endif
