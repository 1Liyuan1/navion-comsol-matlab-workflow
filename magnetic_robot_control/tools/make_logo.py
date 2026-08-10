from pathlib import Path

from PIL import Image


def main() -> None:
    source = Path(r"C:\Users\Liyuan\AppData\Local\Temp\codex-clipboard-b81a15c8-93d1-4d7c-aa6d-e1771d8f2c8e.png")
    destination = Path(__file__).resolve().parents[1] / "assets" / "sysu_logo.png"

    image = Image.open(source).convert("RGBA")
    pixels = []
    for red, green, blue, alpha in image.getdata():
        if max(red, green, blue) < 10:
            pixels.append((red, green, blue, 0))
        else:
            pixels.append((red, green, blue, 255))

    image.putdata(pixels)
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)
    print(destination)


if __name__ == "__main__":
    main()
