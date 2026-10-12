/**
 * @file    app.h
 * @brief   下位机应用层入口。CubeMX 生成的 main.c 只在 USER CODE 区调用这几个函数，
 *          重新生成 CubeMX 代码不会覆盖应用层。
 */
#ifndef APP_H_
#define APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/** 外设初始化完成后调用一次（main.c USER CODE 2） */
void App_Init(void);
/** 主循环里反复调用（main.c USER CODE 3）：串口解析、指示灯、喂狗 */
void App_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
