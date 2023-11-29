from central_hub.central_hub import main
import asyncio

while True:
    try:
        asyncio.run(main())
    except Exception as e:
        print(e)
    finally:
        print(asyncio.current_task().coro)
        asyncio.new_event_loop()
