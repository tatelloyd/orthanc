**Overview**

https://github.com/user-attachments/assets/2d64aded-d729-4b54-8ce3-928435cfa221

This basic robot utlizes focuses on a basic motion control and computer vision to implement an active robotic tower that detects objects,
classifies them, and displays them to the screen.

**Key Features**

-PWM duty cycle controlled servos

-OpenCV library

-YOLOv8 model

-(Optional) Laser pointer toggle

**Bill of Materials**

-RaspberryPi 4: https://vilros.com/products/raspberry-pi-4-model-b-1?variant=40809478750302&country=US&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&tw_source=google&tw_adid=&tw_campaign=19684058556&gad_source=4&gad_campaignid=19684058613&gbraid=0AAAAAD1QJAjEBBM33-wmkODVKD7MW20ge&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJYBMe_ULjNbXKXG4-nAZ0vRzUujs97eeRkRN07h2lzJH7UucQjYrW0aAqtJEALw_wcB

-RaspberryPi 4 charger: https://www.newark.com/raspberry-pi/sc0218/rpi-power-supply-usb-c-5-1v-3a/dp/03AH7034?CMP=KNC-GUSA-PMAX-SHOPPING-ONBOARD-COMP-NEW&mckv=_dc|pcrid||plid||kword||match||slid||product|03AH7034|pgrid||ptaid||&gad_source=1&gad_campaignid=22957739450&gbraid=0AAAAAD5U_g3noAyNJXRbrbLDB7WvxXL9c&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJZuPxSgGmA-fchVQFMMbaFAxiFpXCWWFYP2dVo4Yf00-OqNIMlm94UaAhiNEALw_wcB

-MicroSD card: https://shop.sandisk.com/products/memory-cards/microsd-cards/sandisk-ultra-uhs-i-microsd?sku=SDSQUAC-256G-GN6MA&ef_id=Cj0KCQjw5onGBhDeARIsAFK6QJZq9oXigKwOKMaYQoy68BFhKIx-FvX3doK0AE07Pt_gKbEKBOpIPTEaAn0YEALw_wcB:G:s&s_kwcid=AL!15012!3!!!!x!!!21840826498!&utm_medium=pdsh2&utm_source=gads&utm_campaign=Google-B2C-Conversion-Pmax-NA-US-EN-Memory_Card-All-All-Brand&utm_content=&utm_term=SDSQUAC-256G-GN6MA&cp2=&gad_source=4&gad_campaignid=21836907008&gbraid=0AAAAA-HVYqnR4xOjgBaxD24l-IEJuHxfs&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJZq9oXigKwOKMaYQoy68BFhKIx-FvX3doK0AE07Pt_gKbEKBOpIPTEaAn0YEALw_wcB

-Mini pan/tilt camera platform anti-vibration camera mount: https://www.ebay.com/itm/357458368811?chn=ps&_trkparms=ispr%3D1&amdata=enc%3A1HMn_vsRXSgSVoJ2XC-gUwA37&norover=1&mkevt=1&mkrid=711-117182-37290-0&mkcid=2&mkscid=101&itemid=357458368811&targetid=2321110923741&device=c&mktype=pla&googleloc=9031548&poi=&campaignid=21400684010&mkgroupid=173029508068&rlsatarget=pla-2321110923741&abcId=9448486&merchantid=5564794814&gad_source=4&gad_campaignid=21400684010&gbraid=0AAAAAD_QDh_qJr5uozockIYo-hCpEiqzx&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJYTjEBAq2V94z16V8FpmTMoZqZBRWT4caQj2fBtWKmts73vgPRLlGQaAgVXEALw_wcB

-KY-008 laser pointer: https://www.elecbee.com/en-25585-5Pcs-KY-008-5V-3pin-650nm-Transmitter-Dot-Diode-Copper-Head-Red-Laser-Module-PIC-DIY?utm_term=&utm_campaign=&utm_source=adwords&utm_medium=ppc&hsa_acc=9958698819&hsa_cam=22385142408&hsa_grp=174575468821&hsa_ad=744652661886&hsa_src=g&hsa_tgt=pla-1186173532455&hsa_kw=&hsa_mt=&hsa_net=adwords&hsa_ver=3&gad_source=1&gad_campaignid=22385142408&gbraid=0AAAAADGHwHa_LYr3RN53pNhmzKbeuTyCH&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJbfDRNaVc45qtmBRDwnfKgWxe3wawh8ThkZEbSktR4YYSI0ROTCpk0aAovkEALw_wcB

-Logitech Webcamera: https://www.sweetwater.com/store/detail/C920S--logitech-c920s-pro-hd-1080p-webcam?mrkgadid=&mrkgcl=28&mrkgen=&mrkgbflag=&mrkgcat=&acctid=21700000001645388&dskeywordid=2317218003829&lid=92700080591304514&ds_s_kwgid=58700008754779696&ds_s_inventory_feed_id=97700000007215323&dsproductgroupid=2317218003829&product_id=C920S&prodctry=US&prodlang=en&channel=online&storeid=&device=c&network=g&matchtype=&adpos=largenumber&locationid=9031548&creative=708550342073&targetid=pla-2317218003829&campaignid=21566857885&awsearchcpc=1&gclsrc=aw.ds&gad_source=1&gad_campaignid=21566857885&gbraid=0AAAAAD_RQYkybLXqdJg1RRLOo8RYUCmNH&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJbvDF7Jh0SedGTCL-ehPLzrtalNntmd7aWBQH18b2CX8TkFNVhlWvUaAlGwEALw_wcB

-(Optional) RasberryPi GPIO expansion board: https://www.treedixofficial.com/products/treedix-rpi-gpio-terminal-block-breakout-board-module-expansion-board-compatible-with-raspberry-pi-4b-3b-3b-2b-zero-zero-w?variant=42584983240958&country=US&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&srsltid=AfmBOoo-a8o3SQ5zQ-2aiB8bfsIo50FG7bkaSU9nsPlDt4HaPSbPV9VgRE8

-Jumper cables (as needed)


**Getting Started**
For simplicity, I only used one servos for the pan functionality. I attached the servos into pin 16 (GPIO 23). If you want to attach the
laser pointer, the code is setup to make the attachement at pin 18 (GPIO24). The power lines should both be plugged into 5V source. Obviously,
the software can be modified to change the pin layouts. The GPIO breakout board is if you want to extend the connections with a male to male
connector pin. The board itself isn't necessary.

The stand itself can be put together via this helper video: https://www.youtube.com/watch?v=HUTcWrGf2Hk

The web camera that I used was particularly large relative to the stage, so I didn't include the tilt piece. It may vary depending on the webcamera
that you use. The program itself searches for the USB port, so it should be able to handle any configuration. In my setup I used USB port 0.

**Software Architecture**
The overarching setup is there is a main class, SimpleTurret, that has sub class member for each functionality. These subclasses are:
-SimpleServo: Servos controller
-SimpleLaser: Laser toggle
-SimpleDetector: Computer vision functionality

