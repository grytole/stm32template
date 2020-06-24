# stm32template
Linux/stm32/libopencm3/stm32flash template project
```
sudo apt install git gcc-arm-none-eabi stm32flash

cd ~/dev
git clone https://github.com/grytole/stm32template.git
cd ./stm32template
git submodule add https://github.com/libopencm3/libopencm3.git
git commit -m "import libopencm3"
make -C ./libopencm3
make
```
