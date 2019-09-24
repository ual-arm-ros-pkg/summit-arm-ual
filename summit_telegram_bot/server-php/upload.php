<?php
  $targetdir = '/tmp/summit-uploads/';
  mkdir($targetdir);

  $targetfile = $targetdir.basename($_FILES['UPLOAD']['name']);
  $infile = $_FILES['UPLOAD']['tmp_name'];

  if ($targetdir.$_FILES['UPLOAD']['error'] == UPLOAD_ERR_OK) {
      if (move_uploaded_file($infile, $targetfile)) {
          // file uploaded succeeded
          http_response_code(200);
          echo "OK.<br/>";
      } else {
          // file upload failed
          http_response_code(300);
          echo "ERROR.<br/>";
      }
  }
