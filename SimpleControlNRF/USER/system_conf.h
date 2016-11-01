#ifndef __SYSTEM_CONF_H__
#define __SYSTEM_CONF_H__

/*
 * @brief 调试开关
 *
 * @note 若需要打开调试模式，则保留#define DEBUG语句，打开后，可以通过串口
 *       查看调试信息；若不需要开启，只需把#define DEBUG 语句注释
 *
 */
#define DEBUG

/*
 * @brief 电池类型定义
 *
 * @note 若使用7.4V的电池，需要定义打开 BATTERY_7V4
 *       若需要使用11.1V的电池，需要把以下的行注释
 *
 * @example 7.4V例子:
 *                 #define BATTERY_7V4
 *          11.1V例子:
 *                 // #define BATTERY_7V4
 */
#define BATTERY_7V4


#endif /* __SYSTEM_CONF_H__ */
