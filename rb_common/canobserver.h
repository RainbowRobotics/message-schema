#ifndef ICANOBSERVER_H
#define ICANOBSERVER_H

// ICANObserver: CAN 메시지를 받을 수 있는 객체의 인터페이스
class ICANObserver {
public:
    // 순수 가상 함수: CAN 메시지를 수신했을 때 호출됨
    virtual void onCANMessage(int ch, int id, const unsigned char* data, int dlc) = 0;

    // 가상 소멸자: 파생 클래스의 자원이 올바르게 정리되도록 함
    virtual ~ICANObserver() = default;
};

#endif // ICANOBSERVER_H