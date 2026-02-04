import zenoh

def send_exact_64bytes():
    # 서버 로그에서 캡처한 '정답' 64바이트 데이터
    hex_data = "1000000000000a001000040008000c000a0000000c000000100000001c0000000100000031000000090000004e5f313030303834390000000000000000000000"
    target_data = bytes.fromhex(hex_data)
    
    print(f"--- 검증 완료: {len(target_data)} bytes 생성 ---")
    
    session = zenoh.open(zenoh.Config())
    key = "test/move/goal"
    
    try:
        # Zenoh 1.5.1에서 인자 에러를 피하기 위해 attachment 슬롯 사용 시도
        # 만약 서버가 get의 value를 본다면 value=target_data로 변경
        print(f"'{key}'로 64바이트 전송 중...")
        
        # 1.5.1 구버전 호환성을 위해 최대한 단순하게 호출
        session.put(key, target_data) # 일단 데이터를 경로에 써두고
        session.get(key)              # 서버가 읽어가도록 트리거
        
        print("✅ 전송 완료! 서버 로그를 확인하세요.")
    except Exception as e:
        print(f"❌ 전송 실패: {e}")
    finally:
        session.close()

if __name__ == "__main__":
    send_exact_64bytes()
