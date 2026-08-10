from pathlib import Path

from PIL import Image


def main() -> None:
    source = Path(r"C:\Users\Liyuan\AppData\Local\Temp\codex-clipboard-7f9f6d25-460f-4f4b-90ae-b6dcf3ea4bc0.png")
    destination = Path(__file__).resolve().parents[1] / "assets" / "jiang_lab_logo.png"

    image = Image.open(source).convert("RGBA")
    pixels = []
    for red, green, blue, alpha in image.getdata():
        if red >= 245 and green >= 245 and blue >= 245:
            pixels.append((red, green, blue, 0))
        else:
            pixels.append((red, green, blue, alpha))

    image.putdata(pixels)
    bbox = image.getbbox()
    if bbox is not None:
        image = image.crop(bbox)

    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)
    print(destination)


if __name__ == "__main__":
    main()
