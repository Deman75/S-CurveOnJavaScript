const car = document.querySelector('.car');
const taho = document.querySelector('.taho');

const startTime = Date.now()
const duration = 2000
let needPos = 500

taho.value = 0
taho.value = 500

const anim = () => {
  let progress = ( Date.now() - startTime ) / duration

  if (progress > 1) {
    progress = 1
  }

  progress = Math.sin(Math.acos(progress ** 1.3 - 1))

  taho.value = 500 - (500 * progress)

  if (progress == 1) {
    console.log('done');
    return
  }
  requestAnimationFrame(anim)
}
anim()
