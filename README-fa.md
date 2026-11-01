<div dir="rtl">

# شبیه ساز ربات‌های فوتبالیست پیشرفته

یک شبیه‌سازی فوتبال با چهار ربات که با استفاده از Webots ساخته شده است. این پروژه شامل زمین و توپ اختصاصی، دو تیم آبی و زرد، کنترل‌کننده‌های ربات مبتنی بر Python و یک ناظر مسابقه برای ثبت امتیازها و مدیریت روند بازی است.

> این پروژه از نسخه **R2025a** نرم افزار Webots استفاده می‌کند. برای باز کردن و اجرای world حتما از نسخه سازگار استفاده کنید.

<br>

![Webots simulation world](docs/pic/world.png)

<br>

## قابلیت ها

- دو تیم زرد و آبی که هرکدام شامل دو ربات هستند.
- ربات های تیم زرد: `Y1`, `Y2`
- ربات های تیم آبی: `B1`, `B2`
- مدل ربات‌های چهارچرخ با سیستم حرکتی چند‌جهته (Omnidirectional).
- هر ربات مجهز به حسگرهای GPS، قطب‌نما و سنسور توپ می‌باشد.
- جریمهٔ تماس با دیوار که ربات متخلف را برای مدتی به خارج از زمین منتقل می‌کند.

<br>

## ساختار فایل ها

ساختار فایل‌های پروژه شبیه سازی به صورت زیر می‌باشد.

<div dir="ltr">

```text
.
├── controllers/             # کنترلرها
│   ├── ball_controller/     # کدهای توپ
│   ├── game_supervisor/     # کدهای داور
│   ├── team_blue/           # کدهای تیم آبی
│   └── team_yellow/         # کدهای تیم زرد
├── protos/                    
└── worlds/
    ├── world.wbt            # پروژه شبیه سازی webots
    └── assets/                
```

</div>
<br>


## نرم افزارهای مورد نیاز

| Software Name | Description | Download |
|---|---|---|
| 🐍 Python | زبان برنامه نویسی پایتون. | [Download](https://www.python.org/downloads/) |
| 🤖 Webots | نرم افزار شبیه سازی. | [Download](https://cyberbotics.com/) |
| 💻 Visual Studio Code | نرم افزار ادیتور کد. | [Download](https://code.visualstudio.com/download) |
| 📦 RCJ sim v2 | پروژه شبیه سازی webots. | [Download](https://github.com/fdmxfarhan/rcj-sim-v2/archive/refs/heads/main.zip) |


<br>
<br>

## اجرای شبیه سازی

۱. پس از نصب نرم افزارها و دانلود پروژه `rcj-sim-v2` ابتدا فایل `zip` را از حالت فشرده خارج کنید.

۲. فایل `worlds/world.wbt` را باز کنید.

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/world.gif" alt="Webots simulation world">
</p>

## برنامه ربات‌ها

هر تیم یک کنترلر (`team_yellow`, `team_blue`) دارد که شامل 5 فایل پایتون است.

- `rcj_robot.py`
- `team_blue.py`
- `robot1.py`
- `robot2.py`
- `utils.py`

> توجه داشته باشید که فایل های `rcj_robot.py` و `team_blue.py` نباید تغییر کنند. در مسابقات شبیه سازی تنها فایل های `robot1.py`، `robot2.py` و `utils.py` از شما تحویل گرفته می‌شود.

هر ربات یک فایل پایتون به نام `robot1.py` یا `robot2.py` دارد که الگوریتم‌ها و دستورات آن ربات را شامل می‌شود. همچنین دستورات، توابع و متغیرهایی که به صورت مشترک برای دو ربات تیم استفاده می‌شوند در فایل `utils.py` نوشته می‌شوند. 

برای مثال برنامه ربات 1 را ببینید:

<div dir="ltr">

```python
from utils import *

def loop(rcj):
    ball_pos = rcj.get_ball_position()
    robot_heading = rcj.get_heading()
    robot_pos = rcj.get_position()
    ball_in_kicker = rcj.is_ball_touched()

    if ball_in_kicker:
        rcj.kick()
    if ball_pos:
        ball_angle = getAngle(rcj, ball_pos)
        moveAngle(rcj, ball_angle, 5, robot_heading)
    else:
        rcj.motor(0, 0, 0, 0)
```

</div>

> در این برنامه تمام توابع از فایل `utils.py` به برنامه ربات 1 اضافه می‌شود. در ادامه دستورات این برنامه را بررسی می‌کنیم.

<br>
<br>

1. تابع `loop`: 

این تابع هنگام پس از شروع بازی در یک حلقه بینهایت فراخوانی می‌شود و الگوریتم بازی ربات را اجرا می‌کند.

<div dir="ltr">

```python
def loop(rcj):
```

</div>

2. دریافت مختصات توپ:

با استفاده از تابع زیر می‌توان مختصات توپ **نسبت به ربات** را در یک متغیر ذخیره کرد. خروجی این تابع یک آرایه دو عضوی به صورت `[x, y]` (بر حسب متر) می‌باشد که در آن محور `y` رو به جلوی ربات و محور `x` به سمت چپ ربات است. اگر فاصله توپ از ربات بیشتر از ۰.۸ متر باشد یا ربات دیگری جلوی توپ را گرفته باشد، خروجی `None` خواهد بود.

<div dir="ltr">

```python
ball_pos = rcj.get_ball_position()
```

</div>

برای به دست آوردن زاویه توپ بر حسب درجه (صفر یعنی روبه‌روی ربات و مقدار مثبت یعنی سمت چپ) از تابع `getAngle` در فایل `utils.py` استفاده کنید:

<div dir="ltr">

```python
ball_angle = getAngle(rcj, ball_pos)
```

</div>

3. سنسور قطب نما:

سنسور قطب نما زاویه ربات را بر حسب رادیان می‌دهد. 

<div dir="ltr">

```python
robot_heading = rcj.get_heading()
```

</div>

4. سنسور GPS

با استفاده از تابع زیر می‌توان مختصات ربات در زمین را در یک متغیر ذخیره کرد. خروجی این تابع یک آرایه دو عضوی به صورت `[x, y]` می‌باشد.

<div dir="ltr">

```python
robot_pos = rcj.get_position()
```

</div>

محورهای مختصات زمین به صورت زیر می‌باشد مرکز زمین مبداء مختصات است. مختصات تیم آبی قرینه شده است تا هر دو تیم به سمت +X حمله کنند:

![Webots simulation world](docs/pic/axis.png)

5. سنسور دهانه:

این سنسور تشخیص می‌دهد که توپ در دهانه ربات قرار گرفته یا خیر. تنها در صورتی که توپ در دهانه ربات قرار گرفته باشد امکان شوت کردن توپ یا استفاده از دریبلر وجود دارد

<div dir="ltr">

```python
ball_in_kicker = rcj.is_ball_touched()
```

</div>

## سیستم حرکتی چند جهته

سیستم حرکتی ربات‌های فوتبالیست یک سیستم حرکتی omnidirectional یا چند جهته است. سیستم حرکتی چند جهته به ربات اجازه می‌دهد تا در تمام جهات بتواند حرکت کند. بدین منظور در ربات‌ها از چهار چرخ خورشیدی استفاده شده که می‌توان با استفاده از تابع `rcj.motor(ML1, ML2, MR2, MR1)` سرعت هر موتور را کنترل کرد.

![omni-drive](docs/pic/omni.jpg)

به عنوان مثال برای حرکت ربات به سمت جلو، میتوان از دستور زیر استفاده کرد:

<div dir="ltr">

```python
from utils import *
def loop(rcj):
    rcj.motor(20, 20, -20, -20)
```

</div>

> توجه داشته باشید که حداکثر سرعت هر موتور `20` می‌باشد.

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/forward.gif" alt="Webots simulation world">
</p>


حرکت ربات به سمت راست:

<div dir="ltr">

```python
from utils import *
def loop(rcj):
    rcj.motor(10, -10, -10, 10)

```

</div>

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/right.gif" alt="Webots simulation world">
</p>


حرکت ربات به صورت مورب:

<div dir="ltr">

```python
from utils import *
def loop(rcj):
    rcj.motor(10, 0, -10, 0)

```

</div>

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/orib.gif" alt="Webots simulation world">
</p>

چرخش ربات به دور خود:

<div dir="ltr">

```python
from utils import *
def loop(rcj):
    rcj.motor(5, 5, 5, 5)

```

</div>

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/turn.gif" alt="Webots simulation world">
</p>


</div>
