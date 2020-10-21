/*
 * Utils.h
 *
 *  Created on: 2017-11-29
 *      Author: Administrator
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <jni.h>
#include <utils/RefBase.h>
//#include <binder/Parcel.h>
#include <android/log.h>
using namespace android;
namespace cmdcontrol {
#ifdef __cplusplus
extern "C" {
#endif

#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGV(...)  __android_log_print(ANDROID_LOG_VERBOSE,LOG_TAG,__VA_ARGS__)
#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)


class CmdControlListener : virtual public RefBase {
public:
	virtual void notify(int msg, int ext1, int ext2) = 0;
	virtual ~CmdControlListener(){}
private:
};

#ifdef __cplusplus
}
#endif

}  // namespace cmdcontrol



#endif /* UTILS_H_ */
