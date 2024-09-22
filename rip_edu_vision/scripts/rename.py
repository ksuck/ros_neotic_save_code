import os

# กำหนดเส้นทางไปยังโฟลเดอร์ที่มีไฟล์
folder = '/home/naja/Home/src/rip_edu_vision/scripts/data/captured_images02'

# ดึงรายการไฟล์ทั้งหมดในโฟลเดอร์และกรองเฉพาะไฟล์ที่ลงท้ายด้วย .jpg
files = [f for f in os.listdir(folder) if f.endswith('.jpg')]

# เรียงลำดับไฟล์ตามชื่อ (สามารถปรับเปลี่ยนการเรียงลำดับได้ถ้าต้องการ)
files.sort()

# รีเนมไฟล์พร้อมกับเรียงหมายเลข
for i, filename in enumerate(files):
    new_name = f'image_2{i+1:03d}.jpg'  # เปลี่ยนชื่อไฟล์เป็น image_001.jpg, image_002.jpg, ฯลฯ
    os.rename(os.path.join(folder, filename), os.path.join(folder, new_name))

print("Renaming completed!")
