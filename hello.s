;==============================================================================
; hello.s
; Graham Preston, 11/6/2024
; <gpreston@eou.edu>
;==============================================================================
;
; This sample NES demo is presented as part of my graduate research project
; for CS 407: Seminar at Eastern Oregon University.
; It demonstrates core features of the NES using 6502/2A03 assembly language,
; such as how to render graphics and handle user input.
;
; This code was initially forked from the GitHub repository "example.s",
; https://github.com/bbbradsmith/NES-ca65-example
; created by Brad Smith (rainwarrior), 4/06/2014
; http://rainwarrior.ca

;==============================================================================
; iNES Header
;==============================================================================
.segment "HEADER"
.byte "NES", $1A    ; ID
.byte 2 			; Number of 16k PRG ROM chunks
.byte 1 			; Number of 8k CHR ROM chunk

; Cartridge configuration:
; NROM mapper, vertical nametable mirroring, no PRG RAM
.byte $01, $00

.byte $00			; Default setting PRG RAM (8KB if present)
.byte $00			; NTSC output

; Padding
.byte $00, $00, $00, $00, $00, $00 

;==============================================================================
; CHR ROM
;==============================================================================
.segment "TILES"
.incbin "tiles_left.chr"
.incbin "tiles_right.chr"

;==============================================================================
; NES Memory Map
;==============================================================================
; PPU registers mapped to system address space
REG_PPUCTRL		= $2000
REG_PPUMASK 	= $2001
REG_PPUSTATUS 	= $2002
REG_PPUSCROLL 	= $2005
REG_PPUADDR 	= $2006
REG_PPUDATA 	= $2007
REG_OAMADDR 	= $2003
REG_OAMDATA 	= $2004
REG_OAMDMA 		= $4014

; APU Registers mapped to system address space
REG_DMC			= $4010
REG_APUSTATUS	= $4015	
REG_APUFRAME	= $4017

; Input register mapped to system address space
REG_JOYPAD		= $4016 ; Controller 2 is at REG_JOYPAD + 1

;==============================================================================
; Global Symbols
;==============================================================================
; Default PPU configuration:
; NMI enabled, PPU master, 8x8 sprites, background use left tile table, sprites
; use right table, horizontal VRAM address increment
cfg_PPUCTRL     = %10001000

; Default rendering configuration:
; No colors emphasized, rendering enabled, show graphics in leftmost 8 pixels,
; disable greyscale
cfg_PPUMASK     = %00011110

; Screen and nametables sizes
screen_width    = 256   ; In pixels
screen_height   = 240
nmt_width       = 32    ; In tiles
nmt_height      = 30
nmt_attributes  = 64    ; Size of nametable attributes
nmt_offset      = $0400 ; Memory gap between nametables
tiles_width     = 16    ; Number of tiles per row in CHR ROM

; Important addresses
adr_nmt         = $2000 ; Location of nametable data in PPU memory
adr_nmtattr     = adr_nmt + nmt_offset - nmt_attributes
adr_palette     = $3F00 ; Location of palette data in PPU memory

; Bit masks
mask_scroll     = %00000011 ; Scroll high bits / nametable in PPUCTRL

; Define joypad buttons as bit flags
pad_A           = $1 << 7
pad_B           = $1 << 6
pad_SELECT      = $1 << 5
pad_START       = $1 << 4
pad_U           = $1 << 3
pad_D           = $1 << 2
pad_L           = $1 << 1
pad_R           = $1 << 0

; Misc constants
num_colors      = 32    ; Total number of colors in all palettes
image_start     = 4 + 8 * tiles_width ; Image location in nametable
image_width     = 12    ; In tiles
image_height    = 8
initials_x      = 11    ; Location of initials in tilemap
initials_y      = 30 + nmt_height

; Font locations
font_H      = 0 + 1 * tiles_width
font_E      = 2 + 1 * tiles_width
font_L      = 4 + 1 * tiles_width
font_O      = 6 + 1 * tiles_width
font_W      = 8 + 1 * tiles_width
font_R      = 10 + 1 * tiles_width
font_D      = 12 + 1 * tiles_width
font_C      = 0 + 3 * tiles_width
font_P      = 2 + 3 * tiles_width
font_period = 4 + 3 * tiles_width
font_exclam = 14 + 1 * tiles_width
font_comma  = 6 + 3 * tiles_width
font_star   = 0 + 5 * tiles_width

;==============================================================================
; Interrupt Vectors
;==============================================================================
.segment "VECTORS"
.word nmi
.word reset
.word irq

;==============================================================================
; Reset ISR
; Triggered when the NES is powered on, or when the reset button is pushed.
;==============================================================================
.segment "CODE"
reset:
; Hardware initialization.
    sei       			; Mask interrupts during startup
    lda #0
    sta REG_PPUCTRL 	; Disable PPU NMI
    sta REG_PPUMASK 	; Disable rendering
    sta REG_APUSTATUS 	; Disable APU sound
    sta REG_APUFRAME 	; Disable APU IRQ
    lda #0
    sta REG_DMC 		; Disable DMC IRQ
    cld       			; Disable decimal mode

; It's good practice to initialize the stack pointer.
    ldx #$FF
    txs

; Wait for the first vblank.
    bit REG_PPUSTATUS   ; reset VBL flag
    :
        bit REG_PPUSTATUS
        bpl :-

; Initialize all system RAM.
    lda #0
    ldx #0
    :
        sta $0000, X
        sta $0100, X
        sta $0200, X
        sta $0300, X
        sta $0400, X
        sta $0500, X
        sta $0600, X
        sta $0700, X
        inx
    bne :-

; Place all sprites offscreen.
    lda #$FF
    ldx #0
    :
        sta oam, X
        inx
        inx
        inx
        inx
        bne :-

; Wait for the second vblank.
    :
        bit REG_PPUSTATUS
        bpl :-

; NES is initialized, ready to begin!
; Enable the PPU NMI, and jump to the main program.
    lda #cfg_PPUCTRL
    sta REG_PPUCTRL
    jmp main

;==============================================================================
; NMI ISR
; Triggered when the PPU begins vblank.
;==============================================================================
.segment "ZEROPAGE"
nmi_lock:       .res 1	; When set, prevents NMI re-entry
nmi_ready:      .res 1	; See values below for PPU behavior during NMI
nmi_pass        = 0     ; Skip PPU updates
nmi_update      = 1     ; Update PPU
nmi_disable     = 2     ; Disable PPU

scroll_x:       .res 1	; Scroll position X
scroll_y:       .res 1	; Scroll position Y
scroll_nmt:     .res 1	; Scroll high bits / current nametable

.segment "BSS"
palette:	    .res num_colors ; Current palette buffer
nmt_update_len:	.res 1          ; Current size of nmt_update buffer
; Each entry is 3 bytes in size, and length is reset during NMI ISR.

.segment "NMT"
nmt_update:     .res 256    ; Nametable tile update buffer

.segment "OAM"
oam:            .res 256    ; Sprite OAM data buffer to be uploaded by DMA

.segment "CODE"
nmi:
; Update graphics from buffers in RAM during vblank.

; Save all registers.
    pha
    txa
    pha
    tya
    pha

; Prevent NMI re-entry by checking the lock flag.
    lda nmi_lock
    beq :+
        jmp @nmi_end
    :

; Set NMI re-entry lock flag.
    lda #1
    sta nmi_lock

; Check if PPU needs to be updated, skip ahead otherwise.
    lda nmi_ready
    bne :+ 
        jmp @ppu_update_end
    :

; Turn rendering off if nmi_ready == nmi_disable.
    cmp #nmi_disable
    bne :+
        lda #0
        sta REG_PPUMASK
        ldx #0
        stx nmi_ready
        jmp @ppu_update_end
    :

; Initiate sprite OAM DMA.
    ldx #0
    stx REG_OAMADDR
    lda #>oam
    sta REG_OAMDMA

; Update palette data.
    ; Ensure horizontal nametable increment is set
    lda #cfg_PPUCTRL
    sta REG_PPUCTRL

    ; Set PPU to address palette memory.
    lda REG_PPUSTATUS ; Ensure write is to high byte of REG_PPU_ADDR
    lda #>adr_palette
    sta REG_PPUADDR
    lda #<adr_palette
    sta REG_PPUADDR

    ; Copy palette buffer
    ldx #0 ; Use X register as offset into palette buffer
    :
        lda palette, X
        sta REG_PPUDATA
        inx
        cpx #num_colors
        bcc :-

; Update nametable data with entries from nmt_update buffer.
    ldx #0 ; Use X register as offset into nmt_update buffer

    ; Skip update if buffer is empty.
    cpx nmt_update_len 
    bcs @skip_nmt_update

    ; Read buffer and update nametable memory.
    :
        lda nmt_update, X
        sta REG_PPUADDR
        inx
        lda nmt_update, X
        sta REG_PPUADDR
        inx
        lda nmt_update, X
        sta REG_PPUDATA
        inx
        cpx nmt_update_len
    bcc :-

    ; Reset buffer length.
    lda #0
    sta nmt_update_len
    @skip_nmt_update:

; Handle scrolling.
    ; Update target nametable.
    lda scroll_nmt
    and #mask_scroll    ; Keep only pertinent bits to avoid errors
    ora #cfg_PPUCTRL    ; Keep default PPU configuration
    sta REG_PPUCTRL
    
    ; Update screen coordinates.
    lda scroll_x
    sta REG_PPUSCROLL
    lda scroll_y
    sta REG_PPUSCROLL

; Enable rendering.
    lda #cfg_PPUMASK
    sta REG_PPUMASK

; Rendering complete! Reset PPU update flag.
    ldx #nmi_pass
    stx nmi_ready
    @ppu_update_end:

;
; If this demo had music or sound, this would be a good place to play it.
;

; NMI Complete!
    ; Reset NMI re-entry flag.
    lda #0
    sta nmi_lock

    ; Restore registers.
    @nmi_end:
    pla
    tay
    pla
    tax
    pla
rti

;==============================================================================
; IRQ ISR
;==============================================================================
.segment "CODE"
irq:
rti

;==============================================================================
; PPU Interface Utilities
;==============================================================================
.segment "ZEROPAGE"
temp:   .res 1 ; temp coordinate buffer
tile:   .res 1 ; temp pointer to tile
cursor_tile: .res 1
old_cursor: .res 1

.segment "CODE"
.macro wait_for_nmi
; Wait for the NMI ISR to reset the nmi_ready flag.
	:
		lda nmi_ready
		bne :-
.endmacro

ppu_update:
; Enable rendering during next NMI, and wait until flag is reset in NMI ISR.
	lda #nmi_update 
	sta nmi_ready
    wait_for_nmi
rts

ppu_off:
; Disable rendering during next NMI, and wait until flag is reset in NMI ISR.
	lda #nmi_disable
	sta nmi_ready
    wait_for_nmi
rts

ppu_address_tile:
; Set the address pointer into PPU memory to the location in of a tile in
; nametable memory specified by coordinates in the X and Y registers.
; Ensure that rendering is off before calling this block of code.
;
; X <= nmt_width, X <= 5 bits
; Y <= nmt_height * 4, Y <= 7 bits
; The final address should be be adr_nmt + X + Y * nmt_width, which
; means high byte will be %0010YYYY, and low byte will be %YYYXXXXX.

; Ensure next write to REG_PPUADDR is to high byte.
	lda REG_PPUSTATUS 

; Calculate high byte: %0010YYYY.
	tya
	lsr
	lsr
	lsr
	ora #>adr_nmt
	sta REG_PPUADDR

; Calculate low bye: %YYYXXXXX.
	tya
	asl
	asl
	asl
	asl
	asl
	sta temp
	txa
	ora temp
	sta REG_PPUADDR
rts

ppu_queue_tile_by_coordinates:
; Adds an entry to the nmt_update buffer consisting of the address of a tile in
; nametable memory and the value to set it to.
; Target coordinates are specified by the X and Y registers, and the tile value
; by A.
;
; X <= nmt_width, X <= 5 bits
; Y <= nmt_height * 4, Y <= 7 bits
; The final address should be be adr_nmt + X + Y * nmt_width, which means
; high byte will be %0010YYYY, and low byte will be %YYYXXXXX.

; Protect registers
    sta temp
    pha
    txa
    pha
    tya
    pha

; Temporarily store A (tile value) and X (x coordinate).
    lda temp
	pha
	txa
	pha

; Reuse X as offset into the nmt_update buffer.
	ldx nmt_update_len 

; Calculate high byte: %0010YYYY, and write to buffer.
	tya
	lsr
	lsr
	lsr
	ora #>adr_nmt
	sta nmt_update, X
	inx
    
; Calculate low bye: %YYYXXXXX, and write to buffer.
	tya
	asl
	asl
	asl
	asl
	asl
	sta temp
	pla ; Recover x coordinate
	ora temp
	sta nmt_update, X
	inx

; Write tile value to buffer.
	pla ; Recover tile value
	sta nmt_update, X
	inx

; Record final size of buffer.
	stx nmt_update_len

; Recover registers
    pla
    tay
    pla
    tax
    pla
rts

ppu_queue_tile_by_address:
; Adds an entry to the nmt_update buffer consisting of the address of a tile in
; nametable memory, and the value to set it to.
; Target memory address is directly specified by the Y and X registers, as high
; and low bytes, and the tile value by A.

; Protect registers
    sta temp
    pha
    txa
    pha
    tya
    pha

; Temporarily store A (tile value) and X (low byte).
    lda temp
	pha
	txa
	pha

; Use X as offset into the nmt_update buffer.
	ldx nmt_update_len
	
; Write high byte to buffer.
    tya
	sta nmt_update, X
	inx

; Write low byte to buffer.
	pla ; Recover x value
	sta nmt_update, Y
	inx

; Write tile value to buffer.
	pla ; recover A value (byte)
	sta nmt_update, Y
	inx

; Record final size of buffer.
	stx nmt_update_len

; Recover registers
    pla
    tay
    pla
    tax
    pla
rts

ppu_write_font:
; Draw all four tiles of a font character
    ; Draw top row
    sta tile
    jsr ppu_queue_tile_by_coordinates
    inx
    inc tile
    lda tile
    jsr ppu_queue_tile_by_coordinates

    ; Update coordinate for bottom row
    dec tile
    dex
    lda tile
    clc
    adc #tiles_width
    sta tile
    iny

    ; Draw bottom row
    lda tile
    jsr ppu_queue_tile_by_coordinates

    inx
    inc tile
    lda tile
    jsr ppu_queue_tile_by_coordinates
rts

show_pause_screen:
; While game is paused, scroll to image

; When on the pause screen, stop scrolling.
	lda scroll_x
    cmp #screen_width - 1
	bne :+
        rts
    :

    ; Scroll towards pause screen
    inc scroll_x
    inc scroll_x
    inc scroll_x
    inc scroll_y

    ; Wrap Y at screen_height.
	lda scroll_y
	cmp #screen_height
	bcc :+
		lda #0
		sta scroll_y
	:
rts

reset_scroll:
	lda #0
	sta scroll_x
	sta scroll_y
	sta scroll_nmt
rts

;==============================================================================
; Joypad Utilities
;==============================================================================
.segment "ZEROPAGE"
; Buffer for input in RAM, used with bit masks to determine button states.
joypad: .res 2      ; One byte for each joypad state
pad_held:  .res 2   ; Flags for when buttons are held

.segment "CODE"
poll_input:
; Record current joypad states in RAM.

; Strobe the gamepad control register to latch current button states.
	lda #1
	sta REG_JOYPAD
	sta joypad ; Using first joypad buffer as a ring counter
	lda #0
	sta REG_JOYPAD

; Read 8 bytes from the serial interface, rotating them into the buffer.
	:
		lda REG_JOYPAD + 1
		lsr a
		rol joypad + 1

		lda REG_JOYPAD
		lsr a
		rol joypad
		bcc :-
rts

;==============================================================================
; Main
;==============================================================================
.segment "RODATA"
; Default palette when demo loads.
initial_palette:
.byte $0F, $10, $03, $00    ; bg0 purple/grey
.byte $0F, $10, $03, $27    ; bg1 purple/grey
.byte $0F, $10, $03, $24    ; bg2 purple/grey
.byte $0F, $10, $03, $04    ; bg3 purple/grey
.byte $0F, $18, $28, $38    ; sp0 yellow
.byte $0F, $14, $24, $34    ; sp1 purple
.byte $0F, $1B, $2B, $3B    ; sp2 teal
.byte $0F, $12, $22, $32    ; sp3 marine

.segment "ZEROPAGE"
cursor_x:   .res 1          ; Location of cursor on the screen.
cursor_y:   .res 1
temp_x:     .res 1          ; Temporary storage for coordinates
temp_y:     .res 1
paused:     .res 1          ; Flag for game paused

.segment "CODE"
main:
; Game initialization and main loop.

; Load initial palette into buffer.
	ldx #0
	:
		lda initial_palette, X
		sta palette, X
		inx
		cpx #num_colors
		bcc :-
    
; Draw inital background tiles.
	jsr setup_background

; Center the cursor.
	lda #screen_width / 2
	sta cursor_x
	lda #screen_height / 2
	sta cursor_y

; Set up sprites for cursor.
    lda #15
    sta cursor_tile
	jsr draw_cursor

; Enable rendering and draw screen.
	jsr ppu_update
	
;==============================================================================
@main_loop:
;==============================================================================
; Update status of joypads
	jsr poll_input

; Handle Player 1 input.
	lda joypad
	and #pad_START
	beq :+
		jsr p1_start
	:

	lda joypad
	and #pad_START
	bne :+
        ; Clear start held flag
		lda #pad_START
        eor #$FF
        and pad_held
        sta pad_held
	:

	lda paused
    beq :+
        jsr show_pause_screen
        ; Don't check other buttons
		jmp @skip_input_check 
    :

	lda joypad
	and #pad_U
	beq :+
		jsr p1_u
	:

	lda joypad
	and #pad_D
	beq :+
		jsr p1_d
	:

	lda joypad
	and #pad_L
	beq :+
		jsr p1_l
	:

	lda joypad
	and #pad_R
	beq :+
		jsr p1_r
	:

	lda joypad
	and #pad_SELECT
	beq :+
		jsr p1_select
	:

	lda joypad
	and #pad_SELECT
    bne :+
        ; Clear start held flag
		lda #pad_SELECT
        eor #$FF
        and pad_held
        sta pad_held
    :

	lda joypad
	and #pad_B
	beq :+
		jsr p1_b
	:

	lda joypad
	and #pad_B
	bne :+
        ; Clear start held flag
		lda #pad_B
        eor #$FF
        and pad_held
        sta pad_held
	:

	lda joypad
	and #pad_A
	beq :+
		jsr p1_a
	:

    @skip_input_check:

;
; Player 2 input handling would go here.
;

; Draw everything and finish the frame.
	jsr draw_cursor
	jsr ppu_update

jmp @main_loop

;==============================================================================
; Handlers for button presses.
;==============================================================================
.segment "CODE"
p1_u:
; Move cursor up.
	dec cursor_y
	lda cursor_y

    ; Wrap Y when cursor crosses top of screen.
	cmp #screen_height
	bcc :+
		lda #screen_height - 1
		sta cursor_y
	:
rts

p1_d:
; Move cursor down.
	inc cursor_y
	lda cursor_y

    ; Wrap Y when cursor crosses bottom of screen.
	cmp #screen_height
	bcc :+
		lda #0
		sta cursor_y
	:
rts

p1_l:
	dec cursor_x
rts

p1_r:
	inc cursor_x
rts

p1_select:
; Reset background tiles when select is pressed.
    ; Skip if select is held
    lda pad_held
    and #pad_SELECT
    beq :+
        rts
    :

    ; Set select held flag
    lda pad_held
    ora #pad_SELECT
    sta pad_held

    ; Turn off rendering and reset entire nametable
    jsr ppu_off
    jsr setup_background
rts

p1_start:
; Pause or unpause game when start is pressed.
    ; Skip if start is held
    lda pad_held
    and #pad_START
    beq :+
        rts
    :

    ; Set start held flag
    lda pad_held
    ora #pad_START
    sta pad_held

    ; Toggle pause flag
    lda paused
    eor #$01
    sta paused

    ; If game is paused, show pause message, and hide cursor
    lda paused
    beq :+
        lda cursor_tile
        sta old_cursor
        lda #$FF
        sta cursor_tile
    :

    ; If game is unpaused, reset scroll and hide pause message
    lda paused
    bne :+
        lda old_cursor
        sta cursor_tile
        jsr reset_scroll
    :
rts

p1_b:
    ; Skip if b is held
    lda pad_held
    and #pad_B
    beq :+
        rts
    :

    ; Set b held flag
    lda pad_held
    ora #pad_B
    sta pad_held

    lda cursor_tile
    beq :+
        lda #0
        sta cursor_tile
        rts
    :

    lda #15
    sta cursor_tile
rts

p1_a:
	jsr snap_cursor
	lda cursor_x
    lsr
    lsr
    lsr
    tax ; X = cursor_x / 8
    lda cursor_y
    lsr
    lsr
    lsr
    tay ; Y = cursor_y / 8

    lda cursor_tile
    beq :+
        lda #font_H
        jsr ppu_write_font
        rts
    :

    lda #font_star
    jsr ppu_write_font
rts

;==============================================================================
; Sprite Utilities
;==============================================================================
snap_cursor:
; Snap cursor to nearest tile
	lda cursor_x
	clc
	adc #4
	and #$F8
	sta cursor_x

	lda cursor_y
	clc
	adc #4
	and #$F8
	sta cursor_y

	; Y wraps at bottom of screen
	cmp #screen_height
	bcc :+
		lda #0
		sta cursor_y
	:
rts

draw_cursor:
; four sprites centered around the currently selected tile
; y position (note, needs to be one line higher than sprite's appearance)

; TODO: comment properly
	lda cursor_y
	sec
	sbc #2 ; Y-2
	sta oam+(0*4)+0
	sta oam+(1*4)+0
	lda cursor_y
	clc
	adc #8 ; Y+8
	sta oam+(2*4)+0
	sta oam+(3*4)+0
	; tile
    lda cursor_tile
	sta oam+(0*4)+1
	sta oam+(1*4)+1
	sta oam+(2*4)+1
	sta oam+(3*4)+1
	; attributes
	lda #%00000000 ; no flip
	sta oam+(0*4)+2
	lda #%01000000 ; horizontal flip
	sta oam+(1*4)+2
	lda #%10000000 ; vertical flip
	sta oam+(2*4)+2
	lda #%11000000 ; both flip
	sta oam+(3*4)+2
	; x position
	lda cursor_x
	sec
	sbc #1 ; X-1
	sta oam+(0*4)+3
	sta oam+(2*4)+3
	lda cursor_x
	clc
	adc #9 ; X+9
	sta oam+(1*4)+3
	sta oam+(3*4)+3
rts

;==============================================================================
; Background Utilities
;==============================================================================
.segment "CODE"
setup_background:
; Draw default backgrounds

; Reset first nametable
    ; Set PPU to address first nametable memory
	lda REG_PPUSTATUS ; Ensure write is to high byte of REG_PPU_ADDR
	lda #>adr_nmt
	sta REG_PPUADDR
	lda #<adr_nmt
	sta REG_PPUADDR

	; Reset tile values
	lda #0
	ldy #nmt_height
	:
		ldx #nmt_width
		:
			sta REG_PPUDATA
			dex
			bne :-
		dey
		bne :--
    
	; Set all attributes to 0
    lda #0
	ldx #nmt_attributes
	:
		sta REG_PPUDATA
		dex
		bne :-

; Draw the first background.
	lda #1
	ldy #8 ; start at row 8
	:
		ldx #8  ; start at column 8
        pha     ; Store A
		jsr ppu_address_tile
        pla     ; Recover A
		; wWite a line of checkerboard
		ldx #8
		:
			sta REG_PPUDATA
			eor #$3
			inx
			cpx #(32-8)
			bcc :-
		eor #$3
		iny
		cpy #(30-8)
		bcc :--

; Draw pattern in the second nametable.
    ; Set PPU to address second nametable memory
	lda #>(adr_nmt + nmt_offset)
	sta REG_PPUADDR
	lda #<(adr_nmt + nmt_offset)
	sta REG_PPUADDR

	lda #$00
	ldy #nmt_width
	:
		ldx #nmt_height
		:
			sta REG_PPUDATA
			clc
			adc #1
			and #3
			dex
			bne :-
		clc
		adc #1
		and #3
		dey
		bne :--

; Draw a border around the image.
    ; Initialize coordinates
    ldy #(nmt_height * 2 - image_height - 1)

    ; Draw image.
	:
		ldx #(nmt_width / 2 - image_width / 2 - 1)
		jsr ppu_address_tile
		; Write a line of image
		:
            lda 0;
			sta REG_PPUDATA
			inx
			cpx #(nmt_width / 2 + image_width / 2 + 1)
			bcc :-
		iny
		cpy #(nmt_height * 2 + 6)
		bcc :--

; Draw an image in the second nametable.
    ; Initialize coordinates
    ldy #(nmt_height * 2 - image_height)

    ; Load value of first tile.
	lda #(image_start)
    sta tile;

    ; Draw image.
	:
		ldx #(nmt_width / 2 - image_width / 2)
		jsr ppu_address_tile
		; Write a line of image
		:
            lda tile;
			sta REG_PPUDATA
			inc tile
			inx
			cpx #(nmt_width / 2 + image_width / 2)
			bcc :-
        lda tile
        clc
        adc #(nmt_width / 2 - image_width)
        sta tile
		iny
		cpy #(nmt_height * 2)
		bcc :--

; Draw initials
    ldx #initials_x
    ldy #initials_y
    lda #font_C
    jsr ppu_write_font
    ldx #(initials_x + 2)
    ldy #initials_y
    lda #font_period
    jsr ppu_write_font
    ldx #(initials_x + 3)
    ldy #initials_y
    lda #font_W
    jsr ppu_write_font
    ldx #(initials_x + 5)
    ldy #initials_y
    lda #font_period
    jsr ppu_write_font
    ldx #(initials_x + 6)
    ldy #initials_y
    lda #font_P
    jsr ppu_write_font
    ldx #(initials_x + 8)
    ldy #initials_y
    lda #font_period
    jsr ppu_write_font

; Draw Hello, World!
    ldx #0
    ldy #1
    lda #font_H
    jsr ppu_write_font
    ldx #2
    ldy #1
    lda #font_E
    jsr ppu_write_font
    ldx #4
    ldy #1
    lda #font_L
    jsr ppu_write_font
    ldx #6
    ldy #1
    lda #font_L
    jsr ppu_write_font
    ldx #8
    ldy #1
    lda #font_O
    jsr ppu_write_font
    ldx #10
    ldy #1
    lda #font_comma
    jsr ppu_write_font
    ldx #12
    ldy #1
    lda #font_W
    jsr ppu_write_font
    ldx #14
    ldy #1
    lda #font_O
    jsr ppu_write_font
    ldx #16
    ldy #1
    lda #font_R
    jsr ppu_write_font
    ldx #18
    ldy #1
    lda #font_L
    jsr ppu_write_font
    ldx #20
    ldy #1
    lda #font_D
    jsr ppu_write_font
    ldx #22
    ldy #1
    lda #font_exclam
    jsr ppu_write_font

; 4 stripes of attribute
	lda #>(adr_nmtattr + nmt_offset)
	sta REG_PPUADDR
	lda #<(adr_nmtattr + nmt_offset)
	sta REG_PPUADDR

	lda #0
	ldy #4
	:
		ldx #16
		:
			sta REG_PPUDATA
			dex
			bne :-
		clc
		adc #%01010101
		dey
		bne :--

rts

;==============================================================================
; End of file.
;==============================================================================