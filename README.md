# Autonomous Driving of the Smart Car

Данный проект сделан в рамках проектной программы [Сириус.Лето](https://leto.sirius.online)

## Описание проекта

Современные автомобили стали достаточно умными, чтобы двигаться самостоятельно, однако, безопасность и надежность подобных решений вызывает опасения. Автопилоты допускают ошибки (в том числе, приводящие к травмам пешеходов и ДТП), а также зачастую очень чувствительны к дорожной разметке и дорожной обстановке. Периодически на машинах не срабатывают системы распознавания, или в программу оказывается не заложен алгоритм действий в определённой ситуации

В данном проекте смоделированы условия городской среды для самостоятельного движения роботов-автомобилей с обеспечением безопасности их движения на основе открытого симулятора *gym-duckietown* [(ссылка)](https://github.com/duckietown/gym-duckietown)

## Установка проекта

Перед запуском проекта необходимо установить симулятор *gym-duckietown*. Инструкция по установке находится на официальном репозитории [Duckietown](https://github.com/duckietown/gym-duckietown).

Для установки зависимостей, используемых в проекте, необходимо воспользоваться следующей командой:
```bash
pip3 install -r requirements.txt 
```

Для того, чтобы склонировать репозиторий, можно вспользоваться следующей командой:
```bash
git clone https://github.com/moevm/DT_club_2024.git
```

Для запуска программы необходимо воспользоваться следующей командой:
```bash
python3 main.py
```

## Возможности проекта

В рамках проекта реализованы:

- Вспомогательные методы для работы с камерой: переключение вида камеры, сохранение записи симуляции;
- Определение текущего и целевого положения робота-автомобиля, определение ориентации, ручное и полуавтоматическое управление с помощью клавиш клавиатуры;
- Работа с линиями разметки: выделение контуров с использованием цветовых пространств RGB и HSV, определение положения на изображении через пространственные моменты (`cv2.moments`);
- Автономное движение робота-автомобиля вдоль линии разметки;
- Обработка граничных случаев для обеспечения безопасности: выезд за пределы разметки, остановка перед красной стоп-линией;

## Демонстрация работы
todo

## Участники проекта

<table>
	<tbody>
		<tr>
            <td align="center">
                <a href="https://github.com/AlexanderKamynin">
                    <img src="https://avatars.githubusercontent.com/u/90709676?v=4" width="100;" alt="AlexanderKamynin"/>
                    <br />
                    <sub><b>Alexander Kamynin</b></sub>
                </a>
            </td>
            <td align="center">
                <a href="https://github.com/rooney7L">
                    <img src="https://avatars.githubusercontent.com/u/193333856?v=4" width="100;" alt="rooney7L"/>
                    <br />
                    <sub><b>rooney7L</b></sub>
                </a>
            </td>
            <td align="center">
                <a href="https://github.com/LeoKaplin">
                    <img src="https://avatars.githubusercontent.com/u/193319443?v=4" width="100;" alt="LeoKaplin"/>
                    <br />
                    <sub><b>Leonid Kaplin</b></sub>
                </a>
            </td>
            <td align="center">
                <a href="https://github.com/beast1033">
                    <img src="https://avatars.githubusercontent.com/u/205336709?v=4" width="100;" alt="beast1033"/>
                    <br />
                    <sub><b>Igor Melekhin</b></sub>
                </a>
            </td>
	    <td align="center">
                <a href="https://github.com/ShrekKing">
                    <img src="https://avatars.githubusercontent.com/u/207341529?v=4" width="100;" alt="beast1033"/>
                    <br />
                    <sub><b>Sergej Vinarskij</b></sub>
                </a>
            </td>
	    <td align="center">
                <a href="https://github.com/vitalina-v-v">
                    <img src="https://avatars.githubusercontent.com/u/193338211?v=4" width="100;" alt="vitalina-v-v"/>
                    <br />
                    <sub><b>vitalina</b></sub>
                </a>
            </td>
            <td align="center">
                <a href="https://github.com/KonstantinKondratenko">
                    <img src="https://avatars.githubusercontent.com/u/90711883?v=4" width="100;" alt="KonstantinKondratenko"/>
                    <br />
                    <sub><b>Konstantin Kondratenko</b></sub>
                </a>
            </td>
		</tr>
	<tbody>
</table>
