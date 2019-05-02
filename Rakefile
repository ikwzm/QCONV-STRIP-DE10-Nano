
CC                     = "gcc"
CFLAGS                 = ""
FPGA_BITSTREAM_FILE    = "qconv_strip_axi3.rbf"
DEVICE_TREE_FILE       = "qconv_strip.dts"
DEVICE_TREE_NAME       = "qconv_strip"
DEVICE_TREE_DIRECTORY  = "/config/device-tree/overlays/#{DEVICE_TREE_NAME}"
UIO_DEVICE_NAME        = "uio0"
UDMABUF_DEVICE_FILES   = ["udmabuf-qconv-in", "udmabuf-qconv-out", "udmabuf-qconv-k", "udmabuf-qconv-th"]

desc "Install fpga and devicetrees"
task :install => ["/lib/firmware/#{FPGA_BITSTREAM_FILE}", DEVICE_TREE_FILE] do
  begin
    sh "dtbocfg.rb --install #{DEVICE_TREE_NAME} --dts #{DEVICE_TREE_FILE}"
  rescue => e
    print "error raised:"
    p e
    abort
  end
  if (Dir.exist?(DEVICE_TREE_DIRECTORY) == false)
    abort "can not #{DEVICE_TREE_DIRECTORY} installed."
  end
  device_file = "/dev/" + UIO_DEVICE_NAME
  if (File.exist?(device_file) == false)
    abort "can not #{device_file} installed."
  end
  File::chmod(0666, device_file)
  UDMABUF_DEVICE_FILES.each do |device_file|
    if (File.exist?("/dev/" + device_file) == false)
      abort "can not #{device_file} installed."
    end
    File::chmod(0666, "/dev/" + device_file)
    File::chmod(0666, "/sys/class/udmabuf/" + device_file + "/sync_mode")
    File::chmod(0666, "/sys/class/udmabuf/" + device_file + "/sync_offset")
    File::chmod(0666, "/sys/class/udmabuf/" + device_file + "/sync_size")
    File::chmod(0666, "/sys/class/udmabuf/" + device_file + "/sync_direction")
    File::chmod(0666, "/sys/class/udmabuf/" + device_file + "/sync_owner")
    File::chmod(0666, "/sys/class/udmabuf/" + device_file + "/sync_for_cpu")
    File::chmod(0666, "/sys/class/udmabuf/" + device_file + "/sync_for_device")
  end
end

desc "Uninstall fpga and devicetrees"
task :uninstall do
  if (Dir.exist?(DEVICE_TREE_DIRECTORY) == false)
    abort "can not #{DEVICE_TREE_DIRECTORY} uninstalled: does not already exists."
  end
  sh "dtbocfg.rb --remove #{DEVICE_TREE_NAME}"
end

file "/lib/firmware/#{FPGA_BITSTREAM_FILE}" => ["#{FPGA_BITSTREAM_FILE}"] do
  sh "cp #{FPGA_BITSTREAM_FILE} /lib/firmware/#{FPGA_BITSTREAM_FILE}"
end

directory DEVICE_TREE_DIRECTORY do
  Rake::Task["install"].invoke
end

file "sample1" => ["sample1.c", "sample_common.h"] do
  sh "#{CC} #{CFLAGS} -o sample1 sample1.c"
end
  
file "sample2" => ["sample2.c", "sample_common.h"] do
  sh "#{CC} #{CFLAGS} -o sample2 sample2.c"
end
  
task :default => [DEVICE_TREE_DIRECTORY, "sample1", "sample2"]
