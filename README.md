swrc135b.zip为从TI官网下载的BasicRF工程源码。<br>
source文件夹为从工程源码中提取出的文件夹，在阅读过程中会有些增删。<br>
阅读建议是在阅读源码的过程中参考TI的官方芯片手册，因为会涉及到很多芯片的底层。<br>

三个分支介绍：<br>
CCA-TX:增加了信道检测退避的发射模块，并不完善，但应该可用，参数可能需要调整，周期性发送信号，但仍有些原理没有搞懂。<br>
CCA-noise:用于测试用的干扰信号，理想情况是让这个模块一直发送信号，占用信道，但是实际在信号发送的过程中，信号是一个一个发的，即信号之间有间隔，这可能会造成这些间隔间信道没有被占用，给测试带来一定影响。<br>
CCA-receive:接收信号用