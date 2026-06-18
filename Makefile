TARGET := thumbv7em-none-eabihf
PROFILE := release
PROJECT_DIR := $(CURDIR)
ELF := target/$(TARGET)/$(PROFILE)/rm_robot
BIN := target/$(TARGET)/$(PROFILE)/rm_robot.bin
CARGO := $(HOME)/.cargo/bin/cargo
SBC_MANIFEST := sbc/orange_pi_vision/Cargo.toml

.PHONY: all fmt test clippy clippy-arm sbc-test sbc-clippy check build size flash clean

all: build

fmt:
	$(CARGO) fmt --all -- --check
	$(CARGO) fmt --manifest-path $(SBC_MANIFEST) --all -- --check

test:
	$(CARGO) test --target x86_64-unknown-linux-gnu --lib

clippy:
	$(CARGO) clippy --target x86_64-unknown-linux-gnu --lib -- -D warnings

clippy-arm:
	$(CARGO) clippy --target $(TARGET) --bin rm_robot -- -D warnings

sbc-test:
	$(CARGO) test --manifest-path $(SBC_MANIFEST) --target x86_64-unknown-linux-gnu

sbc-clippy:
	$(CARGO) clippy --manifest-path $(SBC_MANIFEST) --target x86_64-unknown-linux-gnu --all-targets -- -D warnings

check: fmt test clippy clippy-arm sbc-test sbc-clippy build

build:
	cd /tmp && \
		CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUSTFLAGS="-C link-arg=-Tlink.x" \
		$(CARGO) build --manifest-path $(PROJECT_DIR)/Cargo.toml \
		--target $(TARGET) --release
	arm-none-eabi-objcopy -O binary -S $(ELF) $(BIN)
	arm-none-eabi-size $(ELF)

size:
	arm-none-eabi-size $(ELF)

flash: build
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $(ELF) verify reset exit"

clean:
	$(CARGO) clean
