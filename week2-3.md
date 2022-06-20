# Nguyễn Ngọc Vỹ
# **Week 2,3: Tìm hiểu về Mavlink và viết chương trình điều khiển cơ bản cho máy bay sử dụng Mavlink.**
# Mục lục:
1. Nhiệm vụ.
2. Tìm hiểu về Mavlink.
3. Tìm hiểu về Mission Planner và Simulation Quadcopter trong Mission Planner.
4. Tìm hiểu cách giao tiếp giữa Jetson nano và mô phỏng trên Mission Planner.
5. Hướng tiếp cận:
6. Task 1.
7. Task 2.
8. Task 3.
9. Tài liệu tham khảo.
# Nội dung:
## **I. Nhiệm vụ:**
1. Tìm hiểu cơ bản về Mavlink:
    - Mavlink là gì?
    - Các mode của máy bay là gì?
    - Mode nào có thể dùng mavlink để điều khiển máy bay thông qua một chương trình mà không cần sử dụng tay điều khiển?
    - Thư viện nào dùng trong Python để sử dụng Mavlink?
2. Tìm hiểu về Mission Planner và simulation quadcopter trong mission planner:
    - Tìm hiểu cách thiết lập chế độ bay tự động (tạo nhiệm vụ bay trong Mission Planner).
    - Tìm hiểu về chế độ GUIDED trong Mission Planner.
    - Tìm hiểu cách simulation quadcopter iris trong mission planner.
3. Tìm hiểu cách giao tiếp giữa Jetson nano và mô phỏng trên Mission Planner (chạy trên laptop window 10).
4. Viết chương trình cơ bản điều khiển máy bay:
    - Task 1:
        - Cất cánh lên 50m.
        - Bay về phía trước 200m.
        - Bay ngược lại về phía sau 200m.
        - Hạ cánh.
    - Task 2:
        - Lấy dữ liệu GPS hiện tại của máy bay mô phỏng (lat,lon,alt)
        - Kiểm tra xem dữ liệu GPS này đang được cập nhật với tốc độ bao nhiêu lần mỗi giây.
        - Lưu dữ liệu GPS của máy bay ra một file dạng *.text với delimiter là kí tự khoảng trắng (trong 1 phút).
    - Task 3:
        - Tiến hành chạy mô phỏng một nhiệm vụ bay tự động trong Mission planner và lấy GPS của máy bay mô phỏng trong suốt nhiệm vụ đó lưu lại ra một file dạng *.text với delimiter là kí tự khoảng trắng (nhiệm vụ dài ít nhất 1 phút).
5. Công cụ/ từ khóa:
- How to ... in Mission planner  (google search).
- How to..... in python. (google search).
- How to..... in <thư viện python sử dụng với mavlink tìm được>. (google search).
6. Gợi ý:
- Chương trình điều khiển máy bay chạy trên jetson nano sẽ điều khiển máy bay mô phỏng đang được chạy trên laptop thông qua wifi.
7. Các sản phẩm cần hoàn thành:
- 1x báo cáo quá trình tìm hiểu và tổng hợp thông tin về mavlink, mission planner và simulation.
- 1x báo cáo hướng tiếp cận vấn đề cho các chương trình điều khiển máy bay. tụi em có thể sử dụng công cụ http://draw.io để vẽ flowchart.
- video quay màn hình kết quả đã thực hiện được của 3 chương trình
- 3x program ở dạng xxxx.py
## **II. Tìm hiểu về MAVLink:**
1. MAVLink là gì?
    1. Định nghĩa MAVLink:
        - Micro Air Vehicle Link (MAVLink) là một giao thức nhắn tin rất nhẹ để giao tiếp giữa trạm điều khiển mặt đất (Ground Control Station - GCS) với máy bay không người lái và giao tiếp giữa các thành phần của máy bay trên bo mạch.
        - MAVLink được thiết kế như một thư viện sắp xếp thông điệp chỉ tiêu đề (header-only message):
            - MAVLink tuân theo một mẫu thiết kế chuyển vị hiện đại và mô hình thiết kế điểm-điểm: **Các luồng dữ liệu được gửi / xuất bản dưới dạng chủ đề** trong khi các giao thức phụ cấu hình như giao thức nhiệm vụ hoặc giao thức tham số là điểm theo điểm để truyền lại.
            - **Message được xác định trong các tệp XML. Mỗi tệp XML xác định bộ thông báo được hỗ trợ bởi một hệ thống Mavlink cụ thể,** cũng được gọi là "phương ngữ". Bộ thông báo tham chiếu được thực hiện bởi hầu hết các trạm điều khiển mặt đất và máy tự động được xác định trong clom.xml (hầu hết các phương ngữ được xây dựng trên đầu định nghĩa này).
            - **Trình tạo mã tạo các thư viện phần mềm cho các ngôn ngữ lập trình cụ thể từ các định nghĩa tin nhắn XML này,** sau đó có thể được sử dụng bởi máy bay không người lái, trạm điều khiển mặt đất và các hệ thống mavlink khác để giao tiếp. Các thư viện được tạo thường được cấp phép MIT và do đó có thể được sử dụng mà không có giới hạn trong bất kỳ ứng dụng nguồn đóng nào mà không xuất bản mã nguồn của ứng dụng mã nguồn đóng.
        - Mavlink được phát hành lần đầu tiên vào đầu năm 2009 bởi Lorenz Meier và hiện có một số lượng đáng kể những người đóng góp.
    2. Ứng dụng:
        - MAVLink được sử dụng chủ yếu để liên lạc giữa một trạm điều khiển mặt đất (GCS) và các phương tiện không người lái, và trong sự giao tiếp (inter-communication) của hệ thống con của phương tiện. MAVLink có thể được sử dụng để truyền hướng của phương tiện, vị trí và tốc độ GPS của phương tiện đó.
        - Có thể sử dụng MAVLink với nhiều ngôn ngữ lập trình khác nhau (C, C++, Python, C#, Java, JavaScript, Lua, Rust,...), chạy trên nhiều bộ vi điều khiển/hệ điều hành (bao gồm ARM7, ATMEGA, DSPIC, STM32 và Windows, Linux, MacOS, Android và iOS).
    3. Cấu trúc gói:
        - MAVLink 1 chỉ có 8 byte trên mỗi gói, bao gồm dấu hiệu bắt đầu và phát hiện thả gói:
        -   |Field name|Index(Bytes)|Purpose|
            |:---------|:-----------|:------|
            |Start-of-frame|0|Denotes the start of frame transmission (v1.0: 0xFE)|
            |Payload-length|1|length of payload (n)|
            |Packet sequence|2|Each component counts up their send sequence. Allows for detection of packet loss.|
            |System ID|3|Identification of the SENDING system.|Allows to differentiate different systems on the same network.|
            |Component ID|4|Identification of the SENDING component. Allows to differentiate different components of the same system, e.g. the IMU and the autopilot.|
            |Message ID|5|Identification of the message - the id defines what the payload “means” and how it should be correctly decoded.|
            |Payload|6 to (n+6)	|The data into the message, depends on the message id.|
            |CRC	|(n+7) to (n+8)	|Check-sum of the entire packet, excluding the packet start sign (LSB to MSB)|
        - Mavlink 2 chỉ chiếm 14 byte (nhưng là một giao thức an toàn và có thể mở rộng hơn nhiều):
        -   |Field name|Index(Bytes)|Purpose|
            |:---------|:-----------|:------|
            |Start-of-frame	|0	|Denotes the start of frame transmission (v2: 0xFD)|
            |Payload-length	|1	|length of payload (n)
            |Incompatibility flags	|2	|Flags that must be understood for MAVLink compatibility|
            |compatibility flags	|3	|Flags that can be ignored if not understood|
            |Packet sequence	|4	|Each component counts up their send sequence. Allows for detection of packet loss.|
            |System ID	|5	|Identification of the SENDING system. Allows to differentiate different systems on the same network.|
            |Component ID	|6	|Identification of the SENDING component. Allows to differentiate different components of the same system, e.g. the IMU and the autopilot.|
            |Message ID	|7 to 9	|Identification of the message - the id defines what the payload “means” and how it should be correctly decoded.|
            |Payload	|10 to (n+10)	|The data into the message, depends on the message id.|
            |CRC	|(n+11) to (n+12)	|Check-sum of the entire packet, excluding the packet start sign (LSB to MSB)|
            |Signature	|(n+13) to (n+25)	|Signature to verify that messages originate from a trusted source. (optional)|
        - Bởi vì Mavlink không yêu cầu bất kỳ đóng khung bổ sung nào, nó rất phù hợp cho các ứng dụng có băng thông truyền thông rất hạn chế.
2. Các mode của máy bay là gì?
    - Có 25 chế độ bay được tích hợp trên chuyến bay, 10 trong số đó thường xuyên được sử dụng. Có các chế độ để hỗ trợ các cấp độ/loại ổn định chuyến bay khác nhau, tự động tự động tinh vi, hệ thống theo dõi tôi, v.v.
    - Các chế độ bay được điều khiển thông qua đài phát thanh (thông qua công tắc máy phát), thông qua các lệnh truyền giáo hoặc sử dụng các lệnh từ trạm mặt đất (GCS) hoặc máy tính đồng hành.
    - Danh sách 25 mode cùng khả năng của từng loại:
-   |Mode|Alt Ctrl	|Pos Ctrl	|Pos Sensor	|Summary|
    |:---|:---------|:----------|:----------|:------|
    |Acro	|-	|-	|	|Holds attitude, no self-level
    |Airmode	|-	|-/+	|	|Actually not a mode, but a feature,see below
    |Alt Hold	|s	|+	|	|Holds altitude and self-levels the roll & pitch
    |Auto	|A	|A	|Y	|Executes pre-defined mission
    |AutoTune	|s	|A	|Y	|Automated pitch and bank procedure to improve control loops
    |Brake	|A	|A	|Y	|Brings copter to an immediate stop
    |Circle	|s	|A	|Y	|Automatically circles a point in front of the vehicle
    |Drift	|-	|+	|Y	|Like stabilize, but coordinates yaw with roll like a plane
    |Flip	|A	|A	|	|Rises and completes an automated flip
    |FlowHold	|s	|A	|	|Position control using Optical Flow
    |Follow	|s	|A	|Y	|Follows another vehicle
    |Guided	|A	|A	|Y	|Navigates to single points commanded by GCS
    |Heli_Autorotate	|A	|A	|Y	|Used for emergencies in traditional helicopters. Helicopter only. Currently SITL only.
    |Land	|A	|s	|(Y)	|Reduces altitude to ground level, attempts to go straight down
    |Loiter	|s	|s	|Y	|Holds altitude and position, uses GPS for movements
    |PosHold	|s	|+	|Y	|Like loiter, but manual roll and pitch when sticks not centered
    |RTL	|A	|A	|Y	|Returns above takeoff location, may also include landing
    |Simple/Super Simple	|	|	|Y	|An add-on to flight modes to use pilot's view instead of yaw orientation
    |SmartRTL	|A	|A	|Y	|RTL, but traces path to get home
    |Sport	|s	|s	|	|Alt-hold, but holds pitch & roll when sticks centered
    |Stabilize	|-	|+	|	|Self-levels the roll and pitch axis
    |SysID	|-	|+	|	|Special diagnostic/modeling mode
    |Throw	|A	|A	|Y	|Holds position after a throwing takeoff
    |Turtle	|-	|-	|	|Allows reversing and spinning up adjacent pairs of motors in order to flip a crashed, inverted vehicle back upright
    |ZigZag	|A	|A	|Y	|Useful for crop spraying
    - Chú thích kí hiệu:
-   |Symbol	|Definition|
    |:------|:---------|
    |-	|Manual control
    |+	|Manual control with limits & self-level
    |s	|Pilot controls climb rate
    |A	|Automatic control
3. Mode nào có thể dùng mavlink để điều khiển máy bay thông qua một chương trình mà không cần sử dụng tay điều khiển?
    - Các chế độ bay chính điều khiển tự động (Automatic control), không cần sử dụng tay điều khiển (Manual control):
        + AUTO: Follows Mission.
        + LOITER: Circles point where mode switched.
        + CIRCLE: Gently turns aircraft.
        + GUIDED: Circles user defined point from GCS.
        + Return To Launch (RTL): Returns to and circles home or rally point.
        + LAND (AUTO): Final part of automatic mission for touchdown.
    - Chế độ GUIDED là chế độ được khuyến nghị cho máy bay bay tự động mà không cần một nhiệm vụ được xác định trước. Nó cho phép một trạm điều khiển mặt đất (GCS) hoặc máy tính đồng hành điều khiển phương tiện "đang bay" và phản ứng với các sự kiện hoặc tình huống mới khi chúng xảy ra và gửi các lệnh Mavlink để kiểm soát định hướng phương tiện, khu vực quan tâm và phần cứng khác.
4. Thư viện nào dùng trong Python để sử dụng Mavlink?
    - Thư viện có sẵn trên PyPi: Pymavlink.
    - Pymavlink là một thư viện xử lý tin nhắn Mavlink cấp độ thấp và mục đích chung, được viết bằng Python. Nó đã được sử dụng để triển khai truyền thông mavlink trong nhiều loại hệ thống mavlink, bao gồm GCS (MavProxy), API nhà phát triển (DroneKit) và nhiều ứng dụng Mavlink máy tính đồng hành.
    - Thư viện có thể được sử dụng với Python 2.7+ hoặc Python 3.5+ và hỗ trợ cả phiên bản Mavlink 1 và Mavlink 2 của giao thức.
    - Ràng buộc Python để tạo pymavlink và các công cụ và tiện ích hữu ích khác.
    - Nếu bạn đang viết một ứng dụng mavlink để liên lạc với chế độ lái tự động, bạn có thể ưu tiên sử dụng thư viện cấp cao hơn như Mavsdk-Python hoặc DroneKit-Python. Chúng thực hiện một số dịch vụ vi mô của Mavlink.
    - Gói pymavlink bao gồm các mô-đun được tạo cụ thể theo phương ngữ, cung cấp chức năng cấp thấp để mã hóa và giải mã các tin nhắn, và áp dụng và kiểm tra chữ ký.
    -  Hầu hết các nhà phát triển sẽ sử dụng module Mavutil để thiết lập và quản lý kênh truyền thông (vì nó giúp bắt đầu rất dễ dàng). Module này cung cấp các cơ chế đơn giản để thiết lập các liên kết, gửi và nhận tin nhắn và truy vấn một số thuộc tính tự động cơ bản (ví dụ: chế độ bay). Nó cung cấp quyền truy cập vào module phương ngữ được sử dụng để mã hóa/giải mã/ký thông qua thuộc tính (MAV).
    - Các module trong gói pymavlink:
        - Mavutil: dùng để thiết lập các liên kết giao tiếp, nhận và giải mã tin nhắn, chạy các tác vụ định kỳ,...
        - Mavwp: Tải/lưu các điểm tham chiếu, Geofence, điểm Rally.
        - Mavparm: Tải/lưu bộ các tham số Mavlink.
        - Mavextra: Các hàm hữu ích để chuyển đổi các giá trị và thông điệp (ví dụ: mét/thứ hai thành km/h, eulers trong radian từ tứ kết,...).
        - Mavexpression (Nội bộ): Chức năng đánh giá biểu thức Mavlink.
## **III. Tìm hiểu về Mission Planner và Simulation Quadcopter trong Mission Planner:**
1. Tìm hiểu cách thiết lập chế độ bay tự động (tạo nhiệm vụ bay trong Mission Planner):
    - Giao diện Flight Plan:
            <div align='center'>
            <img src="img\plan.png" width='90%'>
            </div>
        - **Bên trái phía trên:** màn hình hiển thị bản đồ gồm điểm Home, các điểm tham chiếu, khoảng cách, quãng đường và quỹ đạo bay.
        - **Bên trái phía dưới:** hiển thị danh sách điểm tham chiếu cho nhiệm vụ. Mô tả kế hoạch của nhiệm vụ theo dạng danh sách các điểm tham chiếu và các sự kiện trên mỗi điểm một cách chi tiết. Ta có thể đặt WPRADIUS, RADIUS LOITER và độ cao mặc định cho các điểm tham chiếu. Độ cao có thể được nhập vào tương đối (dựa trên độ cao điểm Home), tuyệt đối (ASL) và trên địa hình đó. Đơn vị được cài trong Config.
        - **Bên phải:** hiển thị vĩ độ và kinh độ của con trỏ chuột, ASL của con trỏ (từ SRTM - địa hình, dữ liệu). Ta có thể chọn các nhà cung cấp bản đồ khác nhau, tải hoặc lưu các tệp điểm tham chiếu, đọc hoặc viết nhiệm vụ cho Autopilot. Ngoài ra, các chi tiết của vị trí điểm Home trên bản đồ được hiển thị.
    - Chọn bản đồ:
        <div align='center'>
        <img src="img\map.png" width='90%'>
        </div>
    - Tạo điểm Home:
        - Điểm Home là vị trí phương tiện bay được trang bị. Điều đó có nghĩa là khi bạn thực hiện lệnh RTL, phương tiện bay sẽ quay trở lại điểm Home. Hãy trang bị cho phương tiện bay ở vị trí mong muốn nó quay trở lại hoặc sử dụng một điểm rally (điểm tập trung) để thiết lập một điểm trả lại mong muốn.
        - Click trái chuột bất kì ở đâu trên bản đồ sẽ tạo điểm tham chiếu (waypoint) tại điểm đó.
        - Click phải chuột để mở bảng tùy chọn:
            <div align='center'>
            <img src="img\sethome.png" width='90%'>
            </div>
        - Click "Set home here" để chuyển điểm waypoint thành điểm Home
    - Chọn phương tiện bay:
        - Ở góc trái phía trên, chọn tab Simulation:
            <div align='center'>
            <img src="img\simulation.png" width='90%'>
            </div>
        - Chọn model:
            <div align='center'>
            <img src="img\model.png" width='90%'>
            </div>
        - Chọn phương tiện bay:
            <div align='center'>
            <img src="img\quad.png" width='90%'>
            </div>
        - Nếu là lần đầu sử dụng thì ấn "Stable", những lần sau thì cứ ấn "Skip Download" trước khi chọn phương tiện bay:
            <div align='center'>
            <img src="img\stable.png" width='90%'>
            </div>
    - Tạo quỹ đạo bay:
        - Ở góc trái phía trên chọn tab Plan:
            <div align='center'>
            <img src="img\plan.png" width='90%'>
            </div>
        - Tạo nhiệm vụ bay thủ công:
            - Cất cánh tại chỗ (tại điểm Home):
                - Click "Add Below" để tạo điểm 1 trùng điểm Home, Chọn "Take off" cho điểm 1.
                <div align='center'>
                <img src="img\takeoff.png" width='90%'>
                </div>
            - Tạo quỹ đạo bay:
                - Click trái chuột lên các điểm mà ta muốn máy bay đi qua trên bản đồ (waypoint).
                <div align='center'>
                <img src="img\waypoint.png" width='90%'>
                </div>
            - Điểm hạ cánh:
                - Hạ cánh tại điểm mong muốn: Click trái chuột vào điểm muốn hạ cánh, chọn "Land" cho điểm đó.
                - Hạ cánh tại điểm Home: Tạo một điểm bất kì chọn "RETURN TO LAUNCH" cho điểm đó.
                <div align='center'>
                <img src="img\takeoff.png" width='90%'>
                </div>
        - Công cụ tạo nhiệm vụ nâng cao:
            - Mission Planner có khả năng tự động tạo ra nhiều nhiệm vụ khảo sát và loại lưới. Hầu hết đều dựa trên một đa giác được vẽ trên bản đồ.
            - Click vào hình tứ giác màu đen trong góc phải màn hình:
                <div align='center'>
                <img src="img\polygon.png" width='90%'>
                </div>
            - Chọn "Draw a Polygon" rồi vẽ hình đa giác.
            - Click phải chuột, chọn "Auto WP" rồi chọn "Survey (Grid)".
                <div align='center'>
                <img src="img\grid.png" width='90%'>
                </div>
            - Chỉnh sửa theo ý bản thân hoặc để mặc định của hộp thoại, ấn "Accept":
                <div align='center'>
                <img src="img\auto.png" width='90%'>
                </div>
            - Máy tính sẽ tự set up toàn bộ:
                <div align='center'>
                <img src="img\setup.png" width='90%'>
                </div>
        - Click "Write" rồi click "Read"
            <div align='center'>
            <img src="img\write.png" width='90%'>
            </div>
        - Click "OK"
    - Tiến hành bay:
        - Ở góc trái phía trên, chọn tab Data:
            <div align='center'>
            <img src="img\data.png" width='90%'>
            </div>
        - Vào "Actions", chọn "Mission_Start", :
            <div align='center'>
            <img src="img\actions.png" width='90%'>
            </div>
        - Chọn "GUIDED", rồi click "Set Mode" để chọn mode GUIDED.
        - Click "Arm/ Disarm" để khởi động rồi click "Do Action" để cất cánh. 
2. Tìm hiểu về chế độ GUIDED trong Mission Planner:
    - Tổng quan:
        - Chế độ GUIDED không phải là chế độ bay truyền thống.
        - Khả năng của chế độ GUIDED là có thể sử dụng trạm mặt đất (như Mission Planner) và đài phát thanh từ xa (như SIK). Khả năng này cho phép ta tương tác ra lệnh cho phương tiện bay đi đến một vị trí mục tiêu bằng cách nhấp vào một điểm trên bản đồ Data của Mission Planner. Sau khi đạt được vị trí, phương tiện bay sẽ bay giữ nguyên tại vị trí đó, chờ mục tiêu tiếp theo.
    - Yêu cầu:
        - Để sử dụng chế độ GUIDED, ta cần một đài phát thanh từ xa cho phép máy tính và tự động của bạn giao tiếp trong suốt chuyến bay, máy tính trạm mặt đất hoặc máy tính bảng và ứng dụng trạm mặt đất như Mission Planner.
    - Hướng dẫn:
        - Chuẩn bị phương tiện bay trên sân bay và kết nối MAVLink không dây giữa máy bay và laptop.
        - Trên laptop, sử dụng phần mềm đi kèm với module từ xa, đảm bảo rằng nó đang hoạt động và bạn đi khóa 
3. Tìm hiểu cách Simulation Quadcopter Iris trong Mission Planner:
    - Quadcopter là một con Copter có 4 quạt.
    - Cách mô phỏng Quadcopter tương tự như trên đã nêu cách chọn và simulation thiết bị bay.
## **IV. Tìm hiểu về Simulation:**
1. Mô phỏng cho phép thử nghiệm an toàn mã và cài đặt thử nghiệm, và có thể giúp bạn thực hành sử dụng trạm đất của mình mà không phải rời khỏi bàn làm việc.
2. Hầu hết người dùng nên chọn phần mềm trong mô phỏng vòng lặp (SITL) (Dev Wiki) vì nó có thể mô phỏng máy photocopy, máy bay hoặc Rover mà không cần bất kỳ phần cứng xe nào và có thể chạy trên Linux, Windows và Mac OSX. Nó cũng là hình thức mô phỏng dễ nhất để thiết lập.
3.Các bước để mô phỏng thiết bị bay tương tự như trên.
## **V. Hướng tiếp cận:**
1. Task 1 và 2:
    <div align='center'>
    <img src="img\diagram1-2.png" width='90%'>
    </div>
2. Task 3:
    <div align='center'>
    <img src="img\diagram3.png" width='90%'>
    </div>
## **VI. Task 1:**
1. Yêu cầu:
    - Cất cánh lên 50m.
    - Bay về phía trước 200m.
    - Bay ngược lại về phía sau 200m.
    - Hạ cánh.
2. Các bước tiếp cận:
    - Mô phỏng máy bay.
    - Kết nối máy bay.
    - Chọn chế độ bay.
    - Khởi động máy bay.
    - Cất cánh lên 50m.
    - Bay về phía trước 200m.
    - Bay ngược về phía sau 200m.
    - Hạ cánh và tắt máy.
3. Mô phỏng máy bay:
    - **Chú thích:** Mục hướng dẫn chọn và mô phỏng máy bay đã trình bày ở trên.
    - Ta vào Mission Planner.
    - Sau đó vào Simulator để mô phỏng máy bay.
    - Sau đó chọn máy bay.
    - **Lưu ý:** *chú ý hộp thoại ArduCopter.exe*
4. Kết nối với máy bay:
    - Để kết nối với máy bay ta thực hiện function sau:
    ```python
    def connect(connection_str):
            # Start a connection listening to a UDP port
            the_connection = mavutil.mavlink_connection(connection_str)
            # Wait for the first heartbeat
            #   This sets the system and component ID of remote system for the link
            the_connection.wait_heartbeat()
            print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
            # Once connected, use 'the_connection' to get and send messages
            return the_connection
    ```
    - Để lấy được mã kết nối (connection_string):
    - Sau khi chọn máy bay thì sẽ hiện lên hộp thoại ArduCopter.exe.
    - Ta chú ý check xem các cổng kết nối:
    - Ví dụ như ở đây, ta có 3 cổng: TCP:5760, TCP:5762, TCP:5763
        <div align='center'>
        <img src="img\connection.png" width='90%'>
        </div>
    - Ta chọn 1 trong 3 cổng này để kết nối. Ví dụ chọn cổng TCP:5760 thì:
    ```python
    connection_string = 'tcp:localhost:5760'
    the_connection = connect(connection_string)
    ```
    - Sau khi chạy function kết nối, nếu mà output print ra Heartbeat thì có nghĩa là đã kết nối thành công.
    ```
    Heartbeat from system (system 1 component 0)
    ```
5. Chọn chế độ bay:
    - Để chọn chế độ bay, ta thực hiện function sau:
    ```python
    def set_mode(the_connection, mode):
        if mode not in the_connection.mode_mapping():
                print('Unknown mode : {}'.format(mode))
                print('Try:', list(the_connection.mode_mapping().keys()))
                sys.exit(1)
        # Get mode ID
        mode_id = the_connection.mode_mapping()[mode]
        # Set mode
        the_connection.mav.set_mode_send(the_connection.target_system,
                                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        mess(the_connection,'set mode')
    ```
    - Như phần trên cũng đã có đề cập ta sẽ chọn mode GUIDED để tối ưu nhất:
    ```python
    mode = 'GUIDED'
    set_mode(the_connection, mode)
    ```
    - Để biết các function (ví dụ ở đây là function set_mode) có thực hiện thành công hay không, ta thực hiện function sau:
    ```python
    def mess(the_connection, name):
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(name,msg)
    ```
    - Nếu output print ra mà "result : 0" thì có nghĩa là lệnh đã thực hiện thành công, còn nếu "result : 4" thì có nghĩa là thất bại, và nếu không print gì thì có nghĩa là máy bay không nhận được lệnh.
    ```
    set mode COMMAND_ACK {command : 11, result : 0, progress : 0, result_param2 : 0, target_system : 255, target_component : 0}
    ```
6. Khởi động máy bay:
    - Để khởi động máy bay ta thực hiện function sau:
    ```python
    def arm(the_connection,type = 'on'):
        if type == 'on':
            on_off = 1
        elif tyoe == 'off':
            on_off = 0
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                             on_off, 0, 0, 0, 0, 0, 0) #param1 (1: arm, 0: disarm)
        mess(the_connection,'arm')
    ```
    - Trong đó số 1 sau số 0 đầu tiên có nghĩa là bật động cơ, và số số 0 tại vị trí đó có nghĩa là tắt động cơ.
    ```python
    arm(the_connection)
    ```
7. Cất cánh máy bay lên độ cao 50m:
    - Để cất cánh máy bay ta cần thực hiện 3 việc sau:
        1. Lấy GPS home và kiểm tra kết nối tín hiệu gps.
            - Khi ta chưa set home thì mặc định điểm trước khi cất cánh chính là điểm home, ta có thể lấy gps của home bằng cách lấy gps trước khi cất cánh.
            - Ngoài ra việc này còn kiểm tra rằng tín hiệu gps của bạn đã được kết nối trước khi bạn cất cánh. Tránh trường hợp cất cánh lên mới phát hiện gps không hoạt động hoặc không kết nối được hoặc kết nối không ổn định.
        2. Cất cánh lên độ cao mong muốn.
        3. Chờ máy bay cất cánh đến độ cao mong muốn rồi mới gửi lệnh tiếp theo trong nhiệm vụ.
        - **Lưu ý:** *Nhưng do nhiệm vụ 1 không cần đến GPS để thực hiện nên có thể bỏ qua bước 1.*
    - Có 3 cách thực hiện lệnh chờ cất cánh:
        1. Cách sử dụng GPS:
            - Để chờ cho máy bay cất cánh xong, ta chạy vòng lặp vô tận while True: kết hợp với lệnh time.sleep() để liên tục chờ và trả về độ cao GPS mỗi giây đến khi độ cao GPS trả về xấp xỉ bằng với độ cao mong muốn mà ban đầu ta nhập vào (đây cũng chính là lúc máy bay cất cánh xong), ta phá vỡ vòng lặp chờ với lệnh break để tiếp tục nhận cũng lệnh bay khác.
            - Ngoài ra nếu không nhận được GPS thì máy bay tự động phá vòng lặp chờ luôn.
            - Để lấy GPS để thực hiện cách này, ta sử dụng function sau:
            ```python
            def get_gps(the_connection):
                try:
                    gps = the_connection.location(relative_alt = True)
                    return gps, gps.lat, gps.lng, gps.alt
                except:
                    print('no gps')
                    gps = -1
                    return gps, gps, gps, gps
            ```
            - **Ưu điểm:** độ chính xác cao, luôn đúng với mọi độ cao.
            - **Nhược điểm:** code dài, phụ thuộc vào kết nối GPS (kết nối GPS không được hoặc gặp bug với GPS là thất bại nhiệm vụ)
        2. Cách tính toán vật lý:
            - Dựa vào quãng đường chính là độ cao mong muốn và vận tốc máy bay cất cánh cùng gia tốc máy bay cất cánh để tính được thời gian cất cánh.
            - **Ưu điểm:** độ chính xác cao, không phụ thuộc vào GPS.
            - **Nhược điểm:** quá cồng kềnh, rất mất thời gian và rất mất công giải bài toán. Ngoài ra quãng đường dễ dàng có nhưng vận tốc cất cánh và gia tốc cất cánh thì tương đối khó để tìm được, code dài.
        3. Cách thực nghiệm:
            - Cho máy bay bay cất cánh và bấm đồng hồ.
            - Kết quả thu được là để cất cánh lên độ cao 50m thì máy bay mất 30s
            - Đặt lệnh time.sleep(30)
            - **Ưu điểm:** độ chính xác rất cao, bám sát với thực tế, không phụ thuộc vào GPS, vô cùng nhanh gọn lẹ, code rất ngắn.
            - **Nhược điểm:** mỗi độ cao mới là phải đi đo lại.
    - Function hoàn chỉnh cho việc cất cánh phụ thuộc GPS:
    ```python
    def take_off(the_connection, alt):
    # get home gps before take off
        gps, _, _, _ = get_gps(the_connection)
        print('home gps:',gps)
    # --take off--
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                             0, 0, 0, 0, 0, 0, alt)
        mess(the_connection,'take off')
    # -- waiting for taking off --
        while True:
                _, _, _, gps_alt = get_gps(the_connection)
                print(gps_alt)
                time.sleep(1)
                if gps_alt/alt >= 0.999:
                        print('----- taking off is finish -------')
                        break
                if gps_alt == -1:
                        break #error gps
        time.sleep(1) #make sure that taking off is really finish
    ```
    - Chỉ số cuối cùng của lệnh cất cánh chính là độ cao mong muốn tính theo đơn vị là mét.
    - Function cho việc cất cánh không phụ thuộc GPS:
    ```python
    def take_off(the_connection, alt):
    # --take off--
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                             0, 0, 0, 0, 0, 0, alt)
        mess(the_connection,'take off')
    # -- waiting for taking off
        time.sleep(30)
    ```
    - Đối với nhiệm vụ 1 nên xài cách thực nghiệm để tối ưu nhất.
    - Để bay lên độ cao 50m ta thực hiện:
    ```python
    altitude = 50 #meters
    take_off(the_connection, altitude)
    ```
8. Di chuyển đến phía trước 200m:
    - Để di chuyển máy bay một đoạn mà ta mong muốn ta cần làm 2 việc:
        1. Lệnh bay một khoảng cách mong muốn.
        2. Chờ máy bay bay đến điểm đến rồi mới nhận lệnh bay tiếp theo.
    - Có 3 cách để chờ máy bay bay hết đoạn đường:
        1. Cách sử dụng GPS:
            - Khi máy bay hoàn thành xong điểm bay thì máy bay sẽ đứng yên tại chỗ. Khi đó thì GPS tại điểm đó (cụ thể là lat và lon) dường như không đổi. Ta cho chạy vòng lặp vô tân While True: với lệnh chờ time.sleep() và Cứ mỗi 3 giây ta lấy GPS một lần nếu GPS xê dịch ko đáng kể tức là đã 3 giây mà máy bay không có dấu hiệu di chuyển nữa thì ta break vòng lặp.
            - **Nhược điểm:** code dài, phụ thuộc vào kết nối gps, quãng đường nhiệm vụ quá bé so với GPS. Ví dụ ở 2 điểm cách nhau cả mấy trăm mét thì GPS mới cách nhau giữa lat = -35.3633510 và lat = -35.36306850 là rất bé. Nên cách này vẫn chưa hòan thiện.
        2. Cách tính toán vật lý:
            - tương tự như cách tính toán vật lý cất cánh.
        3. Cách thực nghiệm:
            - tương tự như cách thực nghiệm cất cánh. Bay 200m trong vòng 37 giây.
    - Function việc bay một đoạn đường mong muốn phụ thuộc GPS:
    ```python
    def move(the_connection,north ,east ,alt):
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message
                        (10,the_connection.target_system,the_connection.target_component,
                         mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000),
                         north, east, -alt, 0, 0, 0, 0, 0, 0, 0, 0))
        mess(the_connection,'move') 
    #--wait for move--
        time_check = time.time() + 5
        while True:
                _, t1_gps_lat, t1_gps_lon, _ = get_gps(the_connection)
                print(t1_gps_lat)
                time.sleep(3)
                _, t2_gps_lat, t2_gps_lon, _ = get_gps(the_connection)
                print(t2_gps_lat)
                if (t2_gps_lat/t1_gps_lat >= 0.99 and t2_gps_lat/t1_gps_lat <= 1.01) and (t2_gps_lon/t1_gps_lon >= 0.99 and t2_gps_lon/t1_gps_lon <= 1.01) and (time.time() > time_check):
                        print('----- You have reached your destination -----')
                        break
        time.sleep(1)
    ```
    - Trong đó: North là quãng đường mong muốn bay về phía Bắc, East là quãng đường mong muốn bay về phía Đông và Altitude là độ cao bay.
    - Function bay một đoạn đường mong muốn không phụ thuộc gps:
    ```python
    def move(the_connection,north ,east ,alt):
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message
                        (10,the_connection.target_system,the_connection.target_component,
                         mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000),
                         north, east, -alt, 0, 0, 0, 0, 0, 0, 0, 0))
        mess(the_connection,'move')
    #--wait for move--
    time.sleep(37)
    ```
    - Đối với nhiệm vụ 1 nên xài cách thực nghiệm để tối ưu nhất.
    - Để bay tới phía trước 200m, ta thực hiện:
    ```python
    altitude = 50 #meters
    move_north = 200 #meters
    move_east = 0 #meters
    move(the_connection,move_north, move_east, altitude)
    ```
9. Bay ngược về phía sau 200m, hạ cánh và tắt máy.
    - Có 2 cách để thực hiện việc này:
        1. Bay ngược về phía sau 200m và hạ cánh thì chính là điểm home ban đầu nên ta dùng chế độ return to launch là tối ưu nhất.
        2. Bay thủ công bằng lệnh bay về phía Nam 200m, sau đó sử dụng lệnh hạ cánh land.
    - Lệnh bay return to launch và land đã bao gồm việc tắt máy (disarmed) sau khi máy bay hạ cánh thành công
    - Function return to launch:
    ```python
    def return_to_launch(the_connection):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                                             0, 0, 0, 0, 0, 0, 0)
        mess(the_connection,'RTL')
    ```
    - Nếu sử dụng phương án 2 thì ta có function hạ cánh như sau:
    ```python
    def land(the_connection):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
                                             0, 0, 0, 0, 0, 0, 0)
        mess(the_connection,'land')
    ```
    - Để bay về phía sau 200m, hạ cánh và tắt máy, ta thực hiện:
    ```python
    try:
            return_to_launch(the_connection)
    except:
            move(the_connection, -move_north, -move_east, altitude)
            land(the_connection)
    ```
10. **Lỗi:**
    - Có một lỗi giữa lệnh move và get_gps. Khi get_gps xong là lệnh move bị 2 trường hợp:
        1. không thực hiện được lệnh move và các lệnh sau đó như message, wait move, return to launch
        2. thực hiện lệnh move bay đi nhưng không thực hiện các lệnh sau đó.
    - Lỗi này chỉ bị ở máy em còn máy các bạc khác code tương tự thì vẫn chạy bình thường. Lệnh move khi không sử dụng gps thì vẫn di chuyển bình thường. Ngoại trừ lệnh move thì lệnh get_gps không gây ra bất kì lỗi gì đối với các lệnh take_off, return_to_launch, land,...
## **VII. Task 2:**
1. Yêu cầu:
    - Lấy dữ liệu GPS hiện tại của máy bay mô phỏng (lat,lon,alt)
    - Kiểm tra xem dữ liệu GPS này đang được cập nhật với tốc độ bao nhiêu lần mỗi giây.
    - Lưu dữ liệu GPS của máy bay ra một file dạng *.text với delimiter là kí tự khoảng trắng (trong 1 phút).
2. Các bước tiếp cận:
    - Mô phỏng máy bay.
    - Kết nối máy bay.
    - Chọn chế độ bay.
    - Khởi động máy bay.
    - Cất cánh lên 50m.
    - Đếm số lần cập nhật GPS và ghi GPS ra file *.txt trong vòng 1 phút.
    - Tính trung bình số lần cập nhật GPS mỗi giây.
    - Hạ cánh và tắt máy.
3. Mô phỏng máy bay:
    - Tương tự nhiệm vụ 1.
4. Kết nối máy bay:
    - Tương tự nhiệm vụ 1.
5. Chọn chế độ máy bay:
    - Tương tự nhiệm vụ 1.
6. Khởi động máy bay:
    - Tương tự nhiệm vụ 1.
7. Cất cánh máy bay:
    - Tương tự nhiệm vụ 1 (sử dụng cách phụ thuộc GPS).
8. Ghi GPS ra file *.txt trong vòng 1 phút và tính trung bình số lần cập nhật GPS mỗi giây:
    - Công đoạn này cần 3 việc:
        - tạo vòng lặp lặp liên tục trong 1 phút.
        - mỗi lần lặp sẽ lấy gps 1 lần.
        - mỗi lần lấy gps sẽ đếm 1 lần.
        - mỗi lần lấy gps sẽ ghi gps vào file *.txt.
    - Mục tiêu là khi chạy vòng lặp, chạy càng ít lệnh càng tốt, các lệnh càng đơn giản càng tốt để có tốc dộ lặp là tối đa.
    - Để tạo vòng lặp liên tục trong 1 phút, ta thực hiện:
        - tạo một biến thời gian phá vòng lặp (time_out) bằng lấy thời gian trước khi chạy vòng lặp cộng thêm 1 phút.
        - Lặp vô tận while True: đến khi thời gian hiện tại time.time() lớn hơn biến time_out thì phá vòng lặp.
    ```python
    time_wait = 60 #1min = 60 second
    time_out = time.time() + time_wait
    while True:
        if time.time() > time_out:
            break
    ```
    - Lấy GPS bằng function get_gps(the_connection) đã trình bày ở nhiệm vụ 1.
    - In GPS vào file gps.txt gồm 3 bước:
        1. Mở file gps.txt bằng lệnh open với cú pháp:
            ```python
            f = open(path_to_file, mode)
            ```
            - Mode:
                - Chỉ để đọc 'r'
                - Để đọc và viết 'r+'
                - Chỉ để viết 'w'
                - Để viết và đọc 'w+'
                - Chỉ để viết thêm 'a'
                - Để viết thêm và đọc 'a+'
            - Mục tiêu của ta là không xóa đi phần gps cũ mà chỉ thêm gps mới vào. Có thể trong tương lai sẽ sử dụng dữ liệu file gps.txt này nên chọn 'a+' là hợp lý nhất.
            ```python
            try:
                f = open('week2-3\\gps.txt','a+') #open file text for appending text
            except:
                f = open('week2-3\\gps.txt','w+') #open file text for writing text
            print('start get gps to file gps.txt')
            ```
            - Nếu như sau này quá nhiều dữ liệu trong file gps.txt ta muốn clear file mà không cần tìm kiếm location của file gps.txt trong vô vàng file. ta có thể xóa 1 đoạn code trong phần try: khi đó dòng try sẽ bị lỗi và nhảy sang chạy dòng except và open file 'w+'. Với chế độ này máy sẽ xóa sạch gps cũ (tương tự Ctrl A, delete) trước khi lưu dữ liệu gps mới vào.
        2. Viết dữ liệu gps vào file gps.txt bằng lệnh write với cú pháp:
            ```python
            f.write(string1)
            ```
            - Mỗi gps cách nhau bằng một khoảng trắng.
            ```python
            gps, _, _, _ = get_gps(the_connection)
            f.write(str(gps)+' ')
            ```
        3. Đóng file gps.txt với cú pháp:
        ```python
        f.close()
        ```
    - Tổng thể code của công đoạn xử lý GPS này:
    ```python
    def auto_gps(the_connection,time_wait):
        try:
                f = open('week2-3\\gps.txt','a+')
        except:
                f = open('week2-3\\gps.txt','w+')
        count = 0
        print('start get gps to file gps.txt')
        time_out = time.time() + time_wait
        while True:
            if time.time() > time_out:
                break
            gps, _, _, _ = get_gps(the_connection)
            count += 1
            f.write(str(gps)+' ')
        time.sleep(0.1)
        f.write('\n')
        f.close()
        gps_per_second = count / time_wait
        print('----- getting gps is finish -----')
        print(gps_per_second,'gps/second')
    ```
    - Tính trung bình số lần cập nhật GPS mỗi giây:
        - Sau khi đã đếm được số lần cập nhật gps trong 1 phút, ta chia cho thời gian 60 giây là ra được số lần cập nhật gps/giây.
        - Kết quả thu được là 1.05 gps/second
    - **Giải đáp thắc mắc:**
        - Tại sao không chạy 2 luồng, 1 luồng lấy GPS 1 luồng thực hiện nhiệm vụ bay?
            - Nhiệm vụ này chưa cần đến đa luồng vẫn thực hiện được.
            - Khi chạy đa luồng thì GPS trả về là xấp xỉ 0.43 gps/second, thấp hơn 1.05 gps/second.
            - Đa luồng sẽ tối ưu khi lấy GPS từ message hơn thay vì hàm location.
        - Tại sao lại lấy GPS khi máy bay đứng yên trên không thay vì lấy GPS khi máy bay đang bay?
            - Đề không yêu cầu máy bay phải bay đi làm nhiệm vụ khi lấy GPS.
            - Gặp bug về lệnh bay move và lệnh location trong get_gps().
9. Hạ cánh tắt máy bay:
    - Tương tự nhiệm vụ 1.
## **VIII. Task 3:**
1. Yêu cầu:
    - Tiến hành chạy mô phỏng một nhiệm vụ bay tự động trong Mission planner và lấy GPS của máy bay mô phỏng trong suốt nhiệm vụ đó lưu lại ra một file dạng *.text với delimiter là kí tự khoảng trắng (nhiệm vụ dài ít nhất 1 phút).
2. Các bước tiếp cận:
    - Tạo nhiệm vụ bay và lưu nhiệm vụ bay vào file *.waypoints.
    - Mô phỏng máy bay.
    - Kết nối máy bay.
    - Chọn chế độ bay.
    - Khởi động máy bay.
    - Chia 2 luồng:
        1. Luồng nhận GPS:
            - Nhận GPS suốt nhiệm vụ.
            - Ghi GPS nhận được vào file *.txt.
        2. luồng gửi lệnh bay:
            - Cất cánh máy bay.
            - Đọc file *.waypoints.
            - Thực hiện nhiệm vụ trong file *.waypoints.
            - Trở về điểm Home.
            - Hạ cánh và tắt máy.
3. Tạo nhiệm vụ bay và lưu nhiệm vụ bay:
    - Vào tag Plan trong Mission Planner.
    - Tạo nhiệm vụ bay *(đã trình bày ở trên)*.
    - Click vào "Save File" để lưu nhiệm vụ bay vào file *.waypoints.
4. Mô phỏng máy bay:
    - Tương tự nhiệm vụ 1.
5. Kết nối máy bay:
    - Tương tự nhiệm vụ 1.
6. Chọn chế độ bay:
    - Tương tự nhiệm vụ 1.
7. Khởi động máy bay:
    - Tương tự nhiệm vụ 1.
8. Chia 2 luồng:
    - Ta chia 2 luồng:
        - luồng nhận và lưu GPS.
        - luồng gửi lệnh bay.
    - Để phân 2 luồng, ta bỏ mỗi luồng vào một function. Ta có 2 function:
        1. ``` auto_gps3(the_connection) ```
        2. ``` mission3(the_connection,altitude,waypoint_file) ```
    - Code phân luồng:
    ```python
    try:
            t1 = threading.Thread(target = mission3, args = (the_connection,altitude,waypoint_file,))
            t2 = threading.Thread(target = auto_gps3,args = (the_connection,))
            t1.start()
            t2.start()
            t1.join()
            t2.join()
    except:
            print('liu liu ko chay duoc code')
    ```
9. Luồng nhận GPS:
    - Nhận GPS ta thực hiện 3 nhiệm vụ:
        1. Tạo vòng lặp suốt chuyến bay.
        2. Nhận GPS suốt chuyến bay.
        3. Lưu GPS vào file *.txt.
 
    - Tạo vòng lặp suốt chuyến bay:
        - Ta tạo vòng lặp vô tận while True:
        - Khi kết thúc nhiệm vụ cũng là lúc máy bay hạ cánh hoàn thành và tắt máy. Lúc này GPS độ cao của máy bay bằng 0. Ta dựa vào điều này, khi GPS độ cao của máy bay bằng 0 thì break vòng lặp. Nhưng vấn đề xảy ra là trước khi kịp cất cánh độ cao của máy bay bằng 0. Để giải quyết điều này điều kiện của ta là sau khi máy bay cất cánh được 10 giây, nếu GPS độ cao bằng 0 thì nghĩa là máy bay đã hạ cánh và phá vỡ vòng lặp.
    ```python
    time_check = time.time() +10
    while True:
            gps, _, _, alt = get_gps(the_connection)
            if alt < 0.1 and time.time()> time_check:
                    time_end = time.time()
                    break
    time.sleep(0.1)
    ```
    - Nhận GPS suốt chuyến bay đã bao gồm trong vòng lặp trên.
    - Lưu GPS vào file gps_task3.txt tương tự như nhiệm vụ 2.
    - Function đầy đủ của luồng nhận GPS:
    ```python
    def auto_gps3(the_connection):
        try:
                f = open('week2-3\\gps_task3.txt','a+')
        except:
                f = open('week2-3\\gps_task3.txt','w+')
        count = 0
        print('start get gps to file gps_task3.txt')
        time_check = time.time() +10
        while True:
                gps, _, _, alt = get_gps(the_connection)
                count += 1
                f.write(str(gps)+' ')
                if alt < 0.1 and time.time()> time_check:
                        time_end = time.time()
                        break
        time.sleep(0.1)
        f.write('\n')
        f.close()
        time_start = time_check - 10
        time_wait = time_end - time_start
        gps_per_second = count / time_wait
        print('----- getting gps is finish -----')
        print(gps_per_second,'gps/second')
    ```
10. Luồng gửi lệnh bay:
    - Luồng gửi lệnh gồm 4 lệnh:
        - Cất cánh máy bay.
        - Đọc nhiệm vụ từ file *.waypoints.
        - Bay theo nhiệm vụ.
        - Trở về điểm Home, hạ cánh và tắt máy.
    - Cất cánh máy bay:
        - Tương tự nhiệm vụ 2.
    - Đọc nhiệm vụ từ file *.waypoints:
        - Ta sẽ bay đến tọa độ GPS của way point. Nên file *.waypoints trả về ta chỉ qua tâm 3 cột cuối là cột 8, cột 9 và cột 10. Đây là 3 cột chứa dữ liệu GPS (lat, lon, alt) của các điểm way point.
        - Ta thực hiện 4 điều:
            1. Đọc file waypoint.waypoints và lấy dữ liệu của 3 cột cuối
            2. Lưu dữ liệu này vào file waypoint.waypoints.txt để tiện theo dõi về sau.
            3. In dữ liệu này ra màn hình để theo dõi trực tiếp.
            4. Return list GPS của những way point này để làm biến cho lệnh bay thực hiện.
        - Code tổng thể cho function đọc file waypoint.waypoints:
    ```python
    def read_waypoint(file_path='week2-3\\waypoint.waypoints'):
        #This function read *.waypoint
        file = open(file_path, 'r')
        lines = file.readlines()
        pylons = []
        line_count = 1
        # Spit lines into lists
        while line_count < len(lines):
                #print(lines[line_count].split())
                pylons.append(lines[line_count].split())
                line_count += 1
        i = 0
        with open(file_path+'.txt', 'w') as f:
                while i< len(pylons):
                        f.write(pylons[i][8]+" "+pylons[i][9]+" "+pylons[i][10]+"\n")
                        i+=1
                f.close()
        file_path = (file_path+'.txt')
        file = open(file_path, 'r')
        lines = file.readlines()
        pylons = []
        # Spit lines into lists
        for line in lines:
                #print(line)
                #print(line.split())
                pylons.append(line.split())
        print("-------------------------------")
        print("PYLONS:",len(pylons))
        print(pylons)
        print("-------------------------------")
        return pylons
    ```
    - Thực hiện nhiệm vụ trong file *.waypoints:
        - Để thực hiện nhiệm vụ bay ta thực hiện 3 bước:
            - Ta thu được list GPS từ function read_waypoint():
            - Ta chạy vòng lặp để lấy ra từng GPS trong list đó. **Lưu ý:** *Bỏ waypoint đầu tiên nên vòng lặp for sẽ chạy từ 1 đến len()*
            - Mỗi khi nhận được GPS mới, ta tiến hành lệnh bay đến GPS nhận được.
        - Việc bay đến điểm có GPS đó bao gồm 2 công đoạn:
            1. Bay đến điểm có GPS đó.
            2. Chờ đến khi đến điểm đó thì nhận GPS. 
        - Bay đến điểm có GPS đó, ta thực hiện:
        ```python
        destination_gps_lat = int(destination_gps_lat*1e7)
        destination_gps_lon = int(destination_gps_lon*1e7)
        the_connection.mav.set_position_target_global_int_send(0, 
                                                           the_connection.target_system,
                                                           the_connection.target_component,
                                                           mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                           0b110111111000,
                                                           destination_gps_lat, destination_gps_lon, alt,
                                                           0,0,0,
                                                           0,0,0,
                                                           0,0)
        ```
        - Chờ đến khi nhận GPS thì vẫn đang dùng cách thực nghiệm vì chưa triển khai thành công phương pháp phụ thuộc GPS. Điểm yếu là mỗi lần nhập nhiệm vụ bay mới phải khảo sát lại rất cực. Cho trung bình khoảng thời gian bay này là 30 giây cho mỗi đoạn.
        - Function tổng thể cho một chuyến bay đến điểm có GPS mong muốn:
        ```python
        def move_to_gps(the_connection, destination_gps_lat, destination_gps_lon, alt):
        destination_gps_lat = int(destination_gps_lat*1e7)
        destination_gps_lon = int(destination_gps_lon*1e7)
        the_connection.mav.set_position_target_global_int_send(0, 
                                                           the_connection.target_system,
                                                           the_connection.target_component,
                                                           mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                           0b110111111000,
                                                           destination_gps_lat, destination_gps_lon, alt,
                                                           0,0,0,
                                                           0,0,0,
                                                           0,0)
        #wait for move
        time.sleep(30)
        ``` 
    - Trở về điểm Home, hạ cánh và tắt máy:
        - Sử dụng function return_to_launch(the_connection) tương tự nhiệm vụ 1.
## **IX. Tài liệu tham khảo:**
1. https://en.wikipedia.org/wiki/MAVLink
2. https://mavlink.io/en/
3. https://mavlink.io/en/mavgen_python/
4. https://ardupilot.org/plane/docs/flight-modes.html
5. https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
6. https://ardupilot.org/planner/docs/mission-planner-flight-plan.html
7. https://ardupilot.org/planner/docs/common-planning-a-mission-with-waypoints-and-events.html
8. https://ardupilot.org/plane/docs/flight-modes.html
9. https://ardupilot.org/copter/docs/ac2_guidedmode.html#:~:text=The%20guided%20mode%20capability%20is,Mission%20Planner%20Flight%20Data%20map.
10. https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-command-details
11.

