package frc.lib.misc;

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.FileVisitor;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.function.Consumer;
import java.util.function.Supplier;

public final class Util {
  private Util() {}

  public static <T> T returnAfterModifying(T obj, Consumer<? super T> modifier) {
    modifier.accept(obj);
    return obj;
  }

  public static <T> T lazyInitialize(
      Consumer<T> setVariable, Supplier<T> newValueSupplier, T curValue) {
    if (curValue == null) {
      T newValue = newValueSupplier.get();
      setVariable.accept(newValue);
      return newValue;
    }
    return curValue;
  }

  public static <T> Supplier<T> constantSupplierOf(T value) {
    return () -> value;
  }

  public static void deleteDir(Path dir) throws IOException {
    Files.walkFileTree(
        dir,
        new FileVisitor<>() {

          @Override
          public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs)
              throws IOException {
            return FileVisitResult.CONTINUE;
          }

          @Override
          public FileVisitResult postVisitDirectory(Path dir, IOException e) throws IOException {
            if (e != null) {
              throw e;
            }
            Files.delete(dir);
            return FileVisitResult.CONTINUE;
          }

          @Override
          public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
              throws IOException {
            Files.delete(file);
            return FileVisitResult.CONTINUE;
          }

          @Override
          public FileVisitResult visitFileFailed(Path file, IOException e) throws IOException {
            throw e;
          }
        });
  }
}
