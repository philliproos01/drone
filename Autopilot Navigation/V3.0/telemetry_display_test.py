import tkinter as tk
from tkinter import Canvas
import math
import threading
import time
from PIL import Image, ImageDraw, ImageTk

# Helper to get best available resample filter
try:
    from PIL.Image import Resampling
    HAVE_RESAMPLING = True
except ImportError:
    HAVE_RESAMPLING = False

class InstrumentPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Flight Instrument Panel")
        self.geometry("1050x375")  # 1.5x larger
        self.configure(bg='black')

        # Dynamic values
        self.speed = 0
        self.altitude = 0
        self.pitch = 0
        self.roll = 0

        # Scale factor
        self.scale = 1.5

        # Speedometer widget
        self.speed_canvas = Canvas(self, width=int(200 * self.scale), height=int(200 * self.scale), bg='black', highlightthickness=0)
        self.speed_canvas.grid(row=0, column=0, padx=15, pady=30)
        self.speed_img = ImageTk.PhotoImage(Image.new("RGBA", (int(200 * self.scale), int(200 * self.scale)), (0, 0, 0, 255)))
        self.speed_canvas_img = self.speed_canvas.create_image(0, 0, anchor="nw", image=self.speed_img)

        # Altitude widget
        self.altitude_canvas = Canvas(self, width=int(100 * self.scale), height=int(200 * self.scale), bg='black', highlightthickness=0)
        self.altitude_canvas.grid(row=0, column=1, padx=15, pady=30)
        self.altitude_img = ImageTk.PhotoImage(Image.new("RGBA", (int(100 * self.scale), int(200 * self.scale)), (0, 0, 0, 255)))
        self.altitude_canvas_img = self.altitude_canvas.create_image(0, 0, anchor="nw", image=self.altitude_img)

        # Gyroscope widget
        self.gyro_canvas = Canvas(self, width=int(200 * self.scale), height=int(200 * self.scale), bg='black', highlightthickness=0)
        self.gyro_canvas.grid(row=0, column=2, padx=15, pady=30)
        self.gyro_img = ImageTk.PhotoImage(Image.new("RGBA", (int(200 * self.scale), int(200 * self.scale)), (0, 0, 0, 255)))
        self.gyro_canvas_img = self.gyro_canvas.create_image(0, 0, anchor="nw", image=self.gyro_img)

        self._running = True
        self.after(16, self.update_instruments)  # ~60 FPS

    def update_instruments(self):
        self.draw_speedometer()
        self.draw_altitude()
        self.draw_gyroscope()
        if self._running:
            self.after(16, self.update_instruments)

    def draw_speedometer(self):
        s = self.scale
        img = Image.new("RGBA", (int(200 * s), int(200 * s)), (0, 0, 0, 255))
        draw = ImageDraw.Draw(img)
        draw.ellipse([10 * s, 10 * s, 190 * s, 190 * s], outline="white", width=int(2 * s))
        draw.text((60 * s, 5 * s), "Speed (knots)", fill="white")
        for i in range(0, 181, 20):
            angle = math.radians(180 - i)
            x_start = 100 * s + 80 * s * math.cos(angle)
            y_start = 100 * s - 80 * s * math.sin(angle)
            x_end = 100 * s + 90 * s * math.cos(angle)
            y_end = 100 * s - 90 * s * math.sin(angle)
            draw.line([(x_start, y_start), (x_end, y_end)], fill="white", width=int(2 * s))
            tx = 100 * s + 65 * s * math.cos(angle)
            ty = 100 * s - 65 * s * math.sin(angle)
            draw.text((tx - 8 * s, ty - 7 * s), str(i), fill="white")
        speed = self.speed
        angle = math.radians(180 - speed)
        x_needle = 100 * s + 70 * s * math.cos(angle)
        y_needle = 100 * s - 70 * s * math.sin(angle)
        draw.line([(100 * s, 100 * s), (x_needle, y_needle)], fill="red", width=int(3 * s))
        self.speed_img.paste(img)
        # No need to recreate image item, just update the PhotoImage
        self.speed_canvas.itemconfig(self.speed_canvas_img, image=self.speed_img)

    def draw_altitude(self):
        s = self.scale
        img = Image.new("RGBA", (int(100 * s), int(200 * s)), (0, 0, 0, 255))
        draw = ImageDraw.Draw(img)
        draw.rectangle([30 * s, 20 * s, 70 * s, 180 * s], outline="white", width=int(2 * s))
        draw.text((10 * s, 0 * s), "Altitude (ft)", fill="white")
        for i in range(0, 16001, 4000):
            y = 180 * s - (i / 16000) * 160 * s
            draw.line([(30 * s, y), (70 * s, y)], fill="white", width=int(2 * s))
            draw.text((0 * s, y - 7 * s), str(i), fill="white")
        altitude = self.altitude
        y_alt = 180 * s - (altitude / 16000) * 160 * s
        draw.polygon([(70 * s, y_alt), (80 * s, y_alt - 10 * s), (80 * s, y_alt + 10 * s)], fill="red")
        self.altitude_img.paste(img)
        self.altitude_canvas.itemconfig(self.altitude_canvas_img, image=self.altitude_img)

    def draw_center_arrowhead(self, canvas, center_x, center_y):
        s = self.scale
        base_width = 30 * s
        height = 12.5 * s
        canvas.create_polygon(
            center_x - base_width // 2, center_y + height // 2,
            center_x + base_width // 2, center_y + height // 2,
            center_x, center_y - height,
            fill="red"
        )

    def draw_gyroscope(self):
        s = self.scale
        center_x, center_y = int(100 * s), int(100 * s)
        radius = int(90 * s)
        img = Image.new("RGBA", (int(200 * s), int(200 * s)))
        draw = ImageDraw.Draw(img)
        pitch_offset = self.pitch * 2 * s
        sky_bottom = max(0, min(int(200 * s), int(100 * s + pitch_offset)))
        earth_top = sky_bottom
        draw.rectangle([0, 0, int(200 * s), sky_bottom], fill="skyblue")
        draw.rectangle([0, earth_top, int(200 * s), int(200 * s)], fill="saddlebrown")
        # Use Image.Resampling.BICUBIC for Pillow >= 10, fallback for older
        if HAVE_RESAMPLING:
            final_img_rot = img.rotate(-self.roll, center=(center_x, center_y), resample=Resampling.BICUBIC, expand=False)
        else:
            final_img_rot = img.rotate(-self.roll, center=(center_x, center_y), expand=False)
        img = final_img_rot
        mask = Image.new("L", (int(200 * s), int(200 * s)), 0)
        mask_draw = ImageDraw.Draw(mask)
        mask_draw.ellipse([center_x - radius, center_y - radius, center_x + radius, center_y + radius], fill=255)
        img.putalpha(mask)
        ladder_img = Image.new("RGBA", (int(200 * s), int(200 * s)), (0, 0, 0, 0))
        ladder_draw = ImageDraw.Draw(ladder_img)
        # Draw pitch lines and numbers
        ladder_width = 30 * s * 0.5  # 50% narrower
        tick_width = 40 * s * 0.5    # 50% narrower
        number_offset = 10 * s       # much closer to the bars
        for pitch in range(-50, 55, 5):
            if pitch == 0:
                continue
            offset = pitch * 2 * s
            y = center_y + pitch_offset - offset
            if (y < center_y - radius) or (y > center_y + radius):
                continue
            ladder_draw.line([(center_x - ladder_width, y), (center_x + ladder_width, y)], fill="white", width=int(2 * s))
            if pitch % 10 == 0:
                ladder_draw.line([(center_x - tick_width, y), (center_x + tick_width, y)], fill="white", width=int(2 * s))
            ladder_draw.text((center_x - ladder_width - number_offset, y - 7 * s), str(abs(pitch)), fill="white")
            ladder_draw.text((center_x + ladder_width + 2, y - 7 * s), str(abs(pitch)), fill="white")
        if HAVE_RESAMPLING:
            ladder_img_rot = ladder_img.rotate(-self.roll, center=(center_x, center_y), resample=Resampling.BICUBIC, expand=False)
        else:
            ladder_img_rot = ladder_img.rotate(-self.roll, center=(center_x, center_y), expand=False)
        ladder_img = ladder_img_rot
        ladder_img_masked = Image.new("RGBA", (int(200 * s), int(200 * s)), (0, 0, 0, 0))
        ladder_img_masked.paste(ladder_img, (0, 0), mask)
        roll_img = Image.new("RGBA", (int(200 * s), int(200 * s)), (0, 0, 0, 0))
        roll_draw = ImageDraw.Draw(roll_img)
        for angle, label in [(-60, "60"), (-50, ""), (-45, "45"), (-40, ""), (-30, "30"), (-20, ""), (-10, ""), (0, ""), (10, ""), (20, ""), (30, "30"), (40, ""), (45, "45"), (50, ""), (60, "60")]:
            rad = math.radians(angle)
            x1 = center_x + (radius - 5 * s) * math.sin(rad)
            y1 = center_y - (radius - 5 * s) * math.cos(rad)
            x2 = center_x + (radius - 15 * s) * math.sin(rad)
            y2 = center_y - (radius - 15 * s) * math.cos(rad)
            roll_draw.line([(x1, y1), (x2, y2)], fill="white", width=int(2 * s))
            if label:
                lx = center_x + (radius - 25 * s) * math.sin(rad)
                ly = center_y - (radius - 25 * s) * math.cos(rad)
                roll_draw.text((lx, ly), label, fill="white")
        if HAVE_RESAMPLING:
            roll_img_rot = roll_img.rotate(-self.roll, center=(center_x, center_y), resample=Resampling.BICUBIC, expand=False)
        else:
            roll_img_rot = roll_img.rotate(-self.roll, center=(center_x, center_y), expand=False)
        roll_img = roll_img_rot
        roll_img_masked = Image.new("RGBA", (int(200 * s), int(200 * s)), (0, 0, 0, 0))
        roll_img_masked.paste(roll_img, (0, 0), mask)
        final_img = Image.alpha_composite(img, ladder_img_masked)
        final_img = Image.alpha_composite(final_img, roll_img_masked)
        self.gyro_img.paste(final_img)
        self.gyro_canvas.itemconfig(self.gyro_canvas_img, image=self.gyro_img)
        # Draw static overlays (arrow, lines, etc.)
        c = self.gyro_canvas
        c.delete("static")
        c.create_polygon(center_x - 8 * s, center_y - radius + 6 * s, center_x + 8 * s, center_y - radius + 6 * s, center_x, center_y - radius - 8 * s, fill="white", tags="static")
        c.create_line(center_x - 20 * s, center_y + 40 * s, center_x + 20 * s, center_y + 40 * s, fill="white", width=int(4 * s), tags="static")
        c.create_line(center_x, center_y + 40 * s, center_x, center_y + 30 * s, fill="white", width=int(4 * s), tags="static")
        c.create_oval(center_x - 5 * s, center_y + radius - 15 * s, center_x + 5 * s, center_y + radius - 5 * s, fill="white", tags="static")
        c.create_line(center_x, center_y - radius + 10 * s, center_x, center_y + radius - 10 * s, fill="yellow", width=int(2 * s), tags="static")
        self.draw_center_arrowhead(c, center_x, center_y)

    def test_ramp(self):
        def ramp():
            max_speed = 180
            max_altitude = 16000
            max_pitch = 60
            max_roll = 90
            step = 2

            # 1. Ramp up/down speed and altitude, keep gyroscope fixed
            original_pitch = self.pitch
            original_roll = self.roll
            for i in range(0, 101, step):
                self.speed = max_speed * i / 100
                self.altitude = max_altitude * i / 100
                self.pitch = original_pitch
                self.roll = original_roll
                time.sleep(0.02)
            for i in range(100, -1, -step):
                self.speed = max_speed * i / 100
                self.altitude = max_altitude * i / 100
                self.pitch = original_pitch
                self.roll = original_roll
                time.sleep(0.02)

            # 2. Roll left -90°, then right +90°, pitch stays 0
            for i in range(0, 101, step):
                self.roll = -max_roll * i / 100
                self.pitch = 0
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.roll = -max_roll + (2 * max_roll * i / 100)
                self.pitch = 0
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.roll = max_roll - (max_roll * i / 100)
                self.pitch = 0
                time.sleep(0.02)

            # 3. Pitch up +60°, then down -60°, roll stays 0
            self.roll = 0
            for i in range(0, 101, step):
                self.pitch = max_pitch * i / 100
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.pitch = max_pitch - (2 * max_pitch * i / 100)
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.pitch = -max_pitch + (max_pitch * i / 100)
                time.sleep(0.02)

            # Reset pitch and roll
            self.pitch = 0
            self.roll = 0

        threading.Thread(target=ramp, daemon=True).start()

    def on_close(self):
        self._running = False
        self.destroy()

if __name__ == "__main__":
    app = InstrumentPanel()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.after(1000, app.test_ramp)  # Start test ramp after 1 second
    app.mainloop()
